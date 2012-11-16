/*
 *  linux/driver/serial/bbserial_tty.c
 * 
 *  Driver for BrainBoxes cards
 * 
 *  Based on drivers/char/8250.c, by Linus Torvalds, Theodore Ts'o, Russell King.
 *
 *  Copyright (C) 2007 Marcos Lois Bermudez
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * $Id$
 */
#if defined(CONFIG_SERIAL_8250_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/bbserial.h>
#include <linux/nmi.h>
#include <linux/mutex.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include "bbserial.h"

/*
 * Debugging.
 */
#if 0
#define DEBUG_AUTOCONF(fmt...)	printk(fmt)
#else
#define DEBUG_AUTOCONF(fmt...)	do { } while (0)
#endif

#if 0
#define DEBUG_INTR(fmt...)	printk(fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif

#if 0
#define DEBUG_BB(fmt...)	printk(fmt)
#else
#define DEBUG_BB(fmt...)	do { } while (0)
#endif

#define PASS_LIMIT	256

/*
 * We default to IRQ0 for the "no irq" hack.   Some
 * machine types want others as well - they're free
 * to redefine this in their header file.
 */
#define is_real_interrupt(irq)	((irq) != 0)

/*
 * From bbserial_pci.h
 * @todo fix?
 */
extern unsigned int nr_uarts;
extern struct irq_info irq_lists[];
extern struct uart_8250_port bbserial_ports[];
extern struct uart_driver bbserial_reg;

#ifdef CONFIG_SERIAL_8250_DETECT_IRQ
#define CONFIG_SERIAL_DETECT_IRQ 1
#endif
#ifdef CONFIG_SERIAL_8250_MANY_PORTS
#define CONFIG_SERIAL_MANY_PORTS 1
#endif

/*
 * Here we define the default xmit fifo size used for each type of UART.
 */
static const struct bbserial_config uart_config[] = {
	[PORT_UNKNOWN] = {
		.name		= "unknown",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	/*
	 * Detected as PORT_16750 in non-enhaced mode
	 */
	[PORT_BB16PCI958] = {
		.name		= "BB16PCI958",
		.fifo_size	= 16,
		.tx_loadsz	= 16,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
		.flags		= UART_CAP_FIFO | UART_CAP_SLEEP | UART_CAP_AFE,
	},
	/*
	[PORT_16750] = {
		.name		= "TI16750",
		.fifo_size	= 64,
		.tx_loadsz	= 64,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10 |
				  UART_FCR7_64BYTE,
		.flags		= UART_CAP_FIFO | UART_CAP_SLEEP | UART_CAP_AFE,
	}, */
	/*
	 * @todo for other models of BrainBoxes
	 * 
	[PORT_16C950] = {
		.name		= "16C950/954",
		.fifo_size	= 128,
		.tx_loadsz	= 128,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
		.flags		= UART_CAP_FIFO,
	},
	*/
};

static unsigned int serial_in(struct uart_8250_port *up, int offset)
{
	switch (up->port.iotype) {
	case UPIO_MEM:
		return readb(up->port.membase + offset);

	case UPIO_MEM32:
		return readl(up->port.membase + offset);

	default:
		return inb(up->port.iobase + offset);
	}
}

static void
serial_out(struct uart_8250_port *up, int offset, int value)
{
	switch (up->port.iotype) {
	case UPIO_MEM:
		writeb(value, up->port.membase + offset);
		break;

	case UPIO_MEM32:
		writel(value, up->port.membase + offset);
		break;

	default:
		outb(value, up->port.iobase + offset);
	}
}

static void
serial_out_sync(struct uart_8250_port *up, int offset, int value)
{
	switch (up->port.iotype) {
	case UPIO_MEM:
	case UPIO_MEM32:
#ifdef CONFIG_SERIAL_8250_AU1X00
	case UPIO_AU:
#endif
	case UPIO_DWAPB:
		serial_out(up, offset, value);
		serial_in(up, UART_LCR);	/* safe, no side-effects */
		break;
	default:
		serial_out(up, offset, value);
	}
}

/*
 * We used to support using pause I/O for certain machines.  We
 * haven't supported this for a while, but just in case it's badly
 * needed for certain old 386 machines, I've left these #define's
 * in....
 */
#define serial_inp(up, offset)			serial_in(up, offset)
#define serial_outp(up, offset, value)	serial_out(up, offset, value)

/* Uart divisor latch read */
static inline int serial_dl_read(struct uart_8250_port *up)
{
	return serial_inp(up, UART_DLL) | serial_inp(up, UART_DLM) << 8;
}

/* Uart divisor latch write */
static inline void serial_dl_write(struct uart_8250_port *up, int value)
{
	serial_outp(up, UART_DLL, value & 0xff);
	serial_outp(up, UART_DLM, value >> 8 & 0xff);	
}

/*
 * For the 16C950
 */
static void serial_icr_write(struct uart_8250_port *up, int offset, int value)
{
	serial_out(up, UART_SCR, offset);
	serial_out(up, UART_ICR, value);
}

static unsigned int serial_icr_read(struct uart_8250_port *up, int offset)
{
	unsigned int value;

	serial_icr_write(up, UART_ACR, up->acr | UART_ACR_ICRRD);
	serial_out(up, UART_SCR, offset);
	value = serial_in(up, UART_ICR);
	serial_icr_write(up, UART_ACR, up->acr);

	return value;
}

/*
 * For the BB16PCI958
 */
/*
 * _serial_scr_read and _serial_scr_write are for scr outside port address bank
 * @todo create
static unsigned int _serial_scr_read(struct uart_8250_port *up, int offset)
{
	return 0;
}

static void 
_serial_scr_write(struct uart_8250_port *up, int offset, int value)
{

}
*/
static inline unsigned int 
serial_scr_read(struct uart_8250_port *up, int offset) {
	int tmp;
	
	serial_outp(up, UART_MSR, offset);
	tmp = serial_inp(up, UART_SCR);
	serial_outp(up, UART_MSR, 0x00);
	
	return tmp;
}

static inline void 
serial_scr_write(struct uart_8250_port *up, int offset, int value) {
	serial_outp(up, UART_MSR, offset);	// UCR register access
	serial_outp(up, UART_SCR, value);
	serial_outp(up, UART_MSR, 0x00);
}

/*
 * FIFO support.
 */
static inline void bbserial_clear_fifos(struct uart_8250_port *p)
{
	if (p->capabilities & UART_CAP_FIFO) {
		serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO |
			       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_outp(p, UART_FCR, 0);
	}
}

/*
 * IER sleep support.  UARTs which have EFRs need the "extended
 * capability" bit enabled.  Note that on XR16C850s, we need to
 * reset LCR to write to IER.
 */
static inline void bbserial_set_sleep(struct uart_8250_port *p, int sleep)
{
	if (p->capabilities & UART_CAP_SLEEP) {
		if (p->capabilities & UART_CAP_EFR) {
			serial_outp(p, UART_LCR, 0xBF);
			serial_outp(p, UART_EFR, UART_EFR_ECB);
			serial_outp(p, UART_LCR, 0);
		}
		serial_outp(p, UART_IER, sleep ? UART_IERX_SLEEP : 0);
		if (p->capabilities & UART_CAP_EFR) {
			serial_outp(p, UART_LCR, 0xBF);
			serial_outp(p, UART_EFR, 0);
			serial_outp(p, UART_LCR, 0);
		}
	}
}

/*
 * This is a quickie test to see how big the FIFO is.
 * It doesn't work at all the time, more's the pity.
 */
static int size_fifo(struct uart_8250_port *up)
{
	unsigned char old_fcr, old_mcr, old_lcr;
	unsigned short old_dl;
	int count = 0;
	
	old_lcr = serial_inp(up, UART_LCR);
	serial_outp(up, UART_LCR, 0);
	old_fcr = serial_inp(up, UART_FCR);
	old_mcr = serial_inp(up, UART_MCR);
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_outp(up, UART_MCR, UART_MCR_LOOP);
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	old_dl = serial_dl_read(up);
	serial_dl_write(up, 0x0001);
	serial_outp(up, UART_LCR, 0x03);
	for (count = 0; count < 256; count++)
		serial_outp(up, UART_TX, count);
	mdelay(20);/* FIXME - schedule_timeout */
	for (count = 0; (serial_inp(up, UART_LSR) & UART_LSR_DR) &&
	     (count < 256); count++)
		serial_inp(up, UART_RX);
	serial_outp(up, UART_FCR, old_fcr);
	serial_outp(up, UART_MCR, old_mcr);
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	serial_dl_write(up, old_dl);
	serial_outp(up, UART_LCR, old_lcr);

	return count;
}

/*
 * This is a helper routine to autodetect StarTech/Exar/Oxsemi UART's.
 * When this function is called we know it is at least a StarTech
 * 16650 V2, but it might be one of several StarTech UARTs, or one of
 * its clones.  (We treat the broken original StarTech 16650 V1 as a
 * 16550, and why not?  Startech doesn't seem to even acknowledge its
 * existence.)
 *
 * What evil have men's minds wrought...
 */
static void autoconfig_has_efr(struct uart_8250_port *up)
{
	unsigned int id1, id2, id3, rev;

	/*
	 * Everything with an EFR has SLEEP
	 */
	up->capabilities |= UART_CAP_EFR | UART_CAP_SLEEP;

	/*
	 * Check for Oxford Semiconductor 16C950.
	 *
	 * EFR [4] must be set else this test fails.
	 *
	 * This shouldn't be necessary, but Mike Hudson (Exoray@isys.ca)
	 * claims that it's needed for 952 dual UART's (which are not
	 * recommended for new designs).
	 */
	up->acr = 0;
	serial_out(up, UART_LCR, 0xBF);
	serial_out(up, UART_EFR, UART_EFR_ECB);
	serial_out(up, UART_LCR, 0x00);
	id1 = serial_icr_read(up, UART_ID1);
	id2 = serial_icr_read(up, UART_ID2);
	id3 = serial_icr_read(up, UART_ID3);
	rev = serial_icr_read(up, UART_REV);

	DEBUG_AUTOCONF("950id=%02x:%02x:%02x:%02x ", id1, id2, id3, rev);

	if (id1 == 0x16 && id2 == 0xC9 &&
	    (id3 == 0x50 || id3 == 0x52 || id3 == 0x54)) {
		up->port.type = PORT_16C950;

		/*
		 * Enable work around for the Oxford Semiconductor 952 rev B
		 * chip which causes it to seriously miscalculate baud rates
		 * when DLL is 0.
		 */
		if (id3 == 0x52 && rev == 0x01)
			up->bugs |= UART_BUG_QUOT;
		return;
	}
}

/*
 * For debug only
 */
static void bb_dump_scratch(struct uart_8250_port *up) {
	unsigned char sbuff[256];
	unsigned char buff[1024];
	int i;
	

	for(i = 0; i < 256; i++)
		sbuff[i] = serial_scr_read(up, i);
	
	printk("Scratch register:\n");
	for(i = 0; i < 256; i += 16) {
		hex_dump_to_buffer(sbuff + i, 16, 16, 1,
			buff, sizeof(buff), 1);
		printk("  0x%02X: %s\n", i, buff);
	}
}

/*
 * Brainboxes BB16PCI958 detection routine.
 */
static void autoconfig_bb16pci958(struct uart_8250_port *up)
{
	unsigned char save_lcr;
	unsigned char save_ier;
	int save_dl;
	unsigned char ucFifoSize;
	unsigned char X;
	unsigned char ucChipIdentification;
	int i;

	/* Save register state */
	save_lcr = serial_inp(up, UART_LCR);
	save_ier = serial_inp(up, UART_IER);

	/* Perform the detection */
	serial_outp(up, UART_IER, 0x00);
	serial_outp(up, UART_LCR, 0x80);
	serial_outp(up, UART_LCR, 0x00);
	serial_inp(up, UART_IER);

	serial_outp(up, UART_IER, 0x00);
	serial_outp(up, UART_LCR, 0x80);

	serial_outp(up, UART_LCR, 0x00);
	/* 0x10 = (UART_IER_MSI | UART_IER_THRI) */
	if ( (serial_inp(up, UART_IER) & 0x10) != 0x10) {
		DEBUG_BB("%s BB Test 1\n", __FUNCTION__);
		serial_outp(up, UART_LCR, 0x80);

		X = 0x23;

		for ( i = 0 ; i < 42 ; i++ ) {
			serial_outp(up, UART_DLM, X);
			X *= 2;
			X &= 0xfe;
			X |= ((X & 0x80) >> 7) ^ ((X & 0x40) >> 6);
		}

		serial_outp(up, UART_LCR, 0x00 );
		if ((serial_inp(up, UART_IER ) & 0x10) == 0x10) {
			DEBUG_BB("%s BB Test 2\n", __FUNCTION__);
			/* Chip is a Brainboxes UART */
			ucChipIdentification = serial_scr_read(up, BRAINBOXES_SCR_CHIPIDENT);
			DEBUG_BB("%s Chip ident: 0x%02X\n", __FUNCTION__, ucChipIdentification);
			
			/* Enable FIFOs */
			serial_outp(up, UART_FCR,  UART_FCR_ENABLE_FIFO 
				| UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
			DEBUG_BB("%s BB Test 21\n", __FUNCTION__);
			
			/* Maximise FifoSize */
			serial_scr_write(up, BRAINBOXES_SCR_UCR, 
				serial_scr_read(up, BRAINBOXES_SCR_UCR) | BRAINBOXES_SCR_UCR_MAXFIFO);

			DEBUG_BB("%s BB Test 22\n", __FUNCTION__);
				
			/* Read Fifo Size */
			ucFifoSize = serial_scr_read(up, BRAINBOXES_SCR_FIFO_SIZE);
			DEBUG_BB("%s Readed   FIFO Size : %d\n", __FUNCTION__, ucFifoSize);
			//DEBUG_BB("%s Detected FIFO Size : %d\n", __FUNCTION__, size_fifo(up));
			/* @todo fixup for send load not detected, i don't understand this,
			 * if i call size_fifo things work, so i extract the sequence that make
			 * the port redetected if module is unloaded / loaded
			 */
			serial_outp(up, UART_LCR, UART_LCR_DLAB);
			save_dl = serial_dl_read(up);
			serial_dl_write(up, save_dl);
			
			
			DEBUG_BB("%s BB Test 23\n", __FUNCTION__);
			switch(ucFifoSize)
			{
			case 16 :
				up->port.type = PORT_BB16PCI958;
				DEBUG_BB("%s BB Test U16\n", __FUNCTION__);
				break;
			case 32 :
				up->port.type = PORT_BB16PCI958;
				DEBUG_BB("%s BB Test U32\n", __FUNCTION__);
				break;
			case 64 :
				up->port.type = PORT_BB16PCI958;
				DEBUG_BB("%s BB Test U64\n", __FUNCTION__);
				break;
			case 128 :
				up->port.type = PORT_BB16PCI958;
				DEBUG_BB("%s BB Test U1281\n", __FUNCTION__);
				break;
			default :
				DEBUG_BB("%s BB Test Udef U16\n", __FUNCTION__);
				up->port.type = PORT_BB16PCI958;
			}
		}
	}
	
	//bb_dump_scratch(up);

	//
	// Restore register state :
	//
	DEBUG_BB("%s BB Test 3\n", __FUNCTION__);
	serial_outp(up, UART_LCR, save_lcr);
	serial_outp(up, UART_IER, save_ier);
	DEBUG_BB("%s BB Test 4\n", __FUNCTION__);
}


/*
 * We know that the chip has FIFOs.  Does it have an EFR?  The
 * EFR is located in the same register position as the IIR and
 * we know the top two bits of the IIR are currently set.  The
 * EFR should contain zero.  Try to read the EFR.
 */
static void autoconfig_16550a(struct uart_8250_port *up)
{
	unsigned char status1, status2;
	
	DEBUG_BB("%s entry\n", __FUNCTION__);
	up->capabilities |= UART_CAP_FIFO;

	/*
	 * Maybe it requires 0xbf to be written to the LCR.
	 * (other ST16C650V2 UARTs, TI16C752A, etc)
	 */
	serial_outp(up, UART_LCR, 0xBF);
	if (serial_in(up, UART_EFR) == 0) {
		DEBUG_AUTOCONF("EFRv2 ");
		autoconfig_has_efr(up);
		return;
	}

	/*
	 * No EFR.  Try to detect a TI16750, which only sets bit 5 of
	 * the IIR when 64 byte FIFO mode is enabled when DLAB is set.
	 * Try setting it with and without DLAB set.  Cheap clones
	 * set bit 5 without DLAB set.
	 */
	serial_outp(up, UART_LCR, 0);
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR7_64BYTE);
	status1 = serial_in(up, UART_IIR) >> 5;
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR7_64BYTE);
	status2 = serial_in(up, UART_IIR) >> 5;
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_outp(up, UART_LCR, 0);

	DEBUG_AUTOCONF("iir1=%d iir2=%d ", status1, status2);

	if (status1 == 6 && status2 == 7) {
		/* @todo   */
		up->capabilities |= UART_CAP_AFE | UART_CAP_SLEEP;
		/*
		 * Test for Brainboxes Limited propriatory UART chip.
		 * This chip is 750 backwards compatible in non-enhanced
		 * mode.
		 */
		autoconfig_bb16pci958(up);
		return;
	}
}

/*
 * This routine is called by rs_init() to initialize a specific serial
 * port.  It determines what type of UART chip this serial port is
 * using: 8250, 16450, 16550, 16550A.  The important question is
 * whether or not this UART is a 16550A or not, since this will
 * determine whether or not we can use its FIFO features or not.
 */
static void autoconfig(struct uart_8250_port *up, unsigned int probeflags)
{
	unsigned char status1, scratch, scratch2, scratch3;
	unsigned char save_lcr, save_mcr;
	unsigned long flags;

	if (!up->port.iobase && !up->port.mapbase && !up->port.membase)
		return;

	DEBUG_AUTOCONF("ttyB%d: autoconf (0x%04x, 0x%p): ",
			up->port.line, up->port.iobase, up->port.membase);
	DEBUG_BB("%s entry\n", __FUNCTION__);
	/*
	 * We really do need global IRQs disabled here - we're going to
	 * be frobbing the chips IRQ enable register to see if it exists.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	up->capabilities = 0;
	up->bugs = 0;

	if (!(up->port.flags & UPF_BUGGY_UART)) {
		/*
		 * Do a simple existence test first; if we fail this,
		 * there's no point trying anything else.
		 *
		 * 0x80 is used as a nonsense port to prevent against
		 * false positives due to ISA bus float.  The
		 * assumption is that 0x80 is a non-existent port;
		 * which should be safe since include/asm/io.h also
		 * makes this assumption.
		 *
		 * Note: this is safe as long as MCR bit 4 is clear
		 * and the device is in "PC" mode.
		 */
		scratch = serial_inp(up, UART_IER);
		serial_outp(up, UART_IER, 0);
#ifdef __i386__
		outb(0xff, 0x080);
#endif
		/*
		 * Mask out IER[7:4] bits for test as some UARTs (e.g. TL
		 * 16C754B) allow only to modify them if an EFR bit is set.
		 */
		scratch2 = serial_inp(up, UART_IER) & 0x0f;
		serial_outp(up, UART_IER, 0x0F);
#ifdef __i386__
		outb(0, 0x080);
#endif
		scratch3 = serial_inp(up, UART_IER) & 0x0f;
		serial_outp(up, UART_IER, scratch);
		if (scratch2 != 0 || scratch3 != 0x0F) {
			/*
			 * We failed; there's nothing here
			 */
			DEBUG_AUTOCONF("IER test failed (%02x, %02x) ",
				       scratch2, scratch3);
			goto out;
		}
	}

	save_mcr = serial_in(up, UART_MCR);
	save_lcr = serial_in(up, UART_LCR);
	
	/*
	 * Check to see if a UART is really there.  Certain broken
	 * internal modems based on the Rockwell chipset fail this
	 * test, because they apparently don't implement the loopback
	 * test mode.  So this test is skipped on the COM 1 through
	 * COM 4 ports.  This *should* be safe, since no board
	 * manufacturer would be stupid enough to design a board
	 * that conflicts with COM 1-4 --- we hope!
	 */
	if (!(up->port.flags & UPF_SKIP_TEST)) {
		serial_outp(up, UART_MCR, UART_MCR_LOOP | 0x0A);
		status1 = serial_inp(up, UART_MSR) & 0xF0;
		serial_outp(up, UART_MCR, save_mcr);
		if (status1 != 0x90) {
			DEBUG_AUTOCONF("LOOP test failed (%02x) ",
				       status1);
			goto out;
		}
	}

	/*
	 * We're pretty sure there's a port here.  Lets find out what
	 * type of port it is.  The IIR top two bits allows us to find
	 * out if it's 8250 or 16450, 16550, 16550A or later.  This
	 * determines what we test for next.
	 *
	 * We also initialise the EFR (if any) to zero for later.  The
	 * EFR occupies the same register location as the FCR and IIR.
	 */
	serial_outp(up, UART_LCR, 0xBF);
	serial_outp(up, UART_EFR, 0);
	serial_outp(up, UART_LCR, 0);

	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	scratch = serial_in(up, UART_IIR) >> 6;

	DEBUG_AUTOCONF("iir=%d ", scratch);

	switch (scratch) {
	case 3:
		autoconfig_16550a(up);
		break;
	default:
		up->port.type = PORT_UNKNOWN;
		break;
	}

	serial_outp(up, UART_LCR, save_lcr);

	if (up->capabilities != uart_config[up->port.type].flags) {
		printk(KERN_WARNING
		       "ttyB%d: detected caps %08x should be %08x\n",
			up->port.line, up->capabilities,
			uart_config[up->port.type].flags);
	}

	up->port.fifosize = uart_config[up->port.type].fifo_size;
	up->capabilities = uart_config[up->port.type].flags;
	up->tx_loadsz = uart_config[up->port.type].tx_loadsz;

	if (up->port.type == PORT_UNKNOWN)
		goto out;

	/*
	 * Reset the UART.
	 */
	serial_outp(up, UART_MCR, save_mcr);
	bbserial_clear_fifos(up);
	serial_in(up, UART_RX);
	if (up->capabilities & UART_CAP_UUE)
		serial_outp(up, UART_IER, UART_IER_UUE);
	else
		serial_outp(up, UART_IER, 0);

 out:
	spin_unlock_irqrestore(&up->port.lock, flags);
	DEBUG_AUTOCONF("type=%s\n", uart_config[up->port.type].name);
}

static void autoconfig_irq(struct uart_8250_port *up)
{
	unsigned char save_mcr, save_ier;
	unsigned char save_ICP = 0;
	unsigned int ICP = 0;
	unsigned long irqs;
	int irq;

	DEBUG_BB("%s entry\n", __FUNCTION__);
	
	if (up->port.flags & UPF_FOURPORT) {
		ICP = (up->port.iobase & 0xfe0) | 0x1f;
		save_ICP = inb_p(ICP);
		outb_p(0x80, ICP);
		(void) inb_p(ICP);
	}

	/* forget possible initially masked and pending IRQ */
	probe_irq_off(probe_irq_on());
	save_mcr = serial_inp(up, UART_MCR);
	save_ier = serial_inp(up, UART_IER);
	serial_outp(up, UART_MCR, UART_MCR_OUT1 | UART_MCR_OUT2);

	irqs = probe_irq_on();
	serial_outp(up, UART_MCR, 0);
	udelay (10);
	if (up->port.flags & UPF_FOURPORT)  {
		serial_outp(up, UART_MCR,
			    UART_MCR_DTR | UART_MCR_RTS);
	} else {
		serial_outp(up, UART_MCR,
			    UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2);
	}
	serial_outp(up, UART_IER, 0x0f);	/* enable all intrs */
	(void)serial_inp(up, UART_LSR);
	(void)serial_inp(up, UART_RX);
	(void)serial_inp(up, UART_IIR);
	(void)serial_inp(up, UART_MSR);
	serial_outp(up, UART_TX, 0xFF);
	udelay (20);
	irq = probe_irq_off(irqs);

	serial_outp(up, UART_MCR, save_mcr);
	serial_outp(up, UART_IER, save_ier);

	if (up->port.flags & UPF_FOURPORT)
		outb_p(save_ICP, ICP);

	up->port.irq = (irq > 0) ? irq : 0;
}

static inline void __stop_tx(struct uart_8250_port *p)
{
	if (p->ier & UART_IER_THRI) {
		p->ier &= ~UART_IER_THRI;
		serial_out(p, UART_IER, p->ier);
	}
}

static void bbserial_stop_tx(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	
	__stop_tx(up);

	/*
	 * We really want to stop the transmitter from sending.
	 */
	if (up->port.type == PORT_16C950) {
		up->acr |= UART_ACR_TXDIS;
		serial_icr_write(up, UART_ACR, up->acr);
	}
}

static void transmit_chars(struct uart_8250_port *up);

static void bbserial_start_tx(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	
	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);

		if (up->bugs & UART_BUG_TXEN) {
			unsigned char lsr, iir;
			lsr = serial_in(up, UART_LSR);
			up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
			iir = serial_in(up, UART_IIR) & 0x0f;
			if ((up->port.type == PORT_RM9000) ?
				(lsr & UART_LSR_THRE &&
				(iir == UART_IIR_NO_INT || iir == UART_IIR_THRI)) :
				(lsr & UART_LSR_TEMT && iir & UART_IIR_NO_INT))
				transmit_chars(up);
		}
	}

	/*
	 * Re-enable the transmitter if we disabled it.
	 */
	if (up->port.type == PORT_16C950 && up->acr & UART_ACR_TXDIS) {
		up->acr &= ~UART_ACR_TXDIS;
		serial_icr_write(up, UART_ACR, up->acr);
	}
}

static void bbserial_stop_rx(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	
	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
}

static void bbserial_enable_ms(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	
	/* no MSR capabilities */
	if (up->bugs & UART_BUG_NOMSR)
		return;

	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}

static void
receive_chars(struct uart_8250_port *up, unsigned int *status)
{
	struct tty_struct *tty = up->port.info->port.tty;
	unsigned char ch, lsr = *status;
	int max_count = 256;
	char flag;
	
	do {
		ch = serial_inp(up, UART_RX);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		lsr |= up->lsr_saved_flags;
		up->lsr_saved_flags = 0;

		if (unlikely(lsr & UART_LSR_BRK_ERROR_BITS)) {
			/*
			 * For statistics only
			 */
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE)
				up->port.icount.parity++;
			else if (lsr & UART_LSR_FE)
				up->port.icount.frame++;
			if (lsr & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			lsr &= up->port.read_status_mask;

			if (lsr & UART_LSR_BI) {
				DEBUG_INTR("handling break....");
				flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}
		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

		uart_insert_char(&up->port, lsr, UART_LSR_OE, ch, flag);

	ignore_char:
		lsr = serial_inp(up, UART_LSR);
	} while ((lsr & UART_LSR_DR) && (max_count-- > 0));
	spin_unlock(&up->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&up->port.lock);
	*status = lsr;
}

static void transmit_chars(struct uart_8250_port *up)
{
	struct circ_buf *xmit = &up->port.info->xmit;
	int count;
	
	if (up->port.x_char) {
		serial_outp(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&up->port)) {
		bbserial_stop_tx(&up->port);
		return;
	}
	if (uart_circ_empty(xmit)) {
		__stop_tx(up);
		return;
	}

	count = up->tx_loadsz;
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	DEBUG_INTR("THRE...");

	if (uart_circ_empty(xmit))
		__stop_tx(up);
}

static unsigned int check_modem_status(struct uart_8250_port *up)
{
	unsigned int status = serial_in(up, UART_MSR);
	
	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.info != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

		wake_up_interruptible(&up->port.info->delta_msr_wait);
	}

	return status;
}

/*
 * This handles the interrupt from one port.
 */
static inline void
bbserial_handle_port(struct uart_8250_port *up)
{
	unsigned int status;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);

	status = serial_inp(up, UART_LSR);

	DEBUG_INTR("status = %x...", status);

	if (status & UART_LSR_DR)
		receive_chars(up, &status);
	check_modem_status(up);
	if (status & UART_LSR_THRE)
		transmit_chars(up);

	spin_unlock_irqrestore(&up->port.lock, flags);
}

/*
 * This is the serial driver's interrupt routine.
 *
 * Arjan thinks the old way was overly complex, so it got simplified.
 * Alan disagrees, saying that need the complexity to handle the weird
 * nature of ISA shared interrupts.  (This is a special exception.)
 *
 * In order to handle ISA shared interrupts properly, we need to check
 * that all ports have been serviced, and therefore the ISA interrupt
 * line has been de-asserted.
 *
 * This means we need to loop through all ports. checking that they
 * don't have an interrupt pending.
 */
static irqreturn_t bbserial_interrupt(int irq, void *dev_id)
{
	struct irq_info *i = dev_id;
	struct list_head *l, *end = NULL;
	int pass_counter = 0, handled = 0;

	DEBUG_INTR("bbserial_interrupt(%d)...", irq);

	spin_lock(&i->lock);

	l = i->head;
	do {
		struct uart_8250_port *up;
		unsigned int iir;

		up = list_entry(l, struct uart_8250_port, list);

		iir = serial_in(up, UART_IIR);
		if (!(iir & UART_IIR_NO_INT)) {
			bbserial_handle_port(up);

			handled = 1;

			end = NULL;
		} else if (up->port.iotype == UPIO_DWAPB &&
			  (iir & UART_IIR_BUSY) == UART_IIR_BUSY) {
			/* The DesignWare APB UART has an Busy Detect (0x07)
			 * interrupt meaning an LCR write attempt occured while the
			 * UART was busy. The interrupt must be cleared by reading
			 * the UART status register (USR) and the LCR re-written. */
			unsigned int status;
			status = *(volatile u32 *)up->port.private_data;
			serial_out(up, UART_LCR, up->lcr);

			handled = 1;

			end = NULL;
		} else if (end == NULL)
			end = l;

		l = l->next;

		if (l == i->head && pass_counter++ > PASS_LIMIT) {
			/* If we hit this, we're dead. */
			printk(KERN_ERR "serial8250: too much work for "
				"irq%d\n", irq);
			break;
		}
	} while (l != end);

	spin_unlock(&i->lock);

	DEBUG_INTR("end.\n");

	return IRQ_RETVAL(handled);
}

/*
 * To support ISA shared interrupts, we need to have one interrupt
 * handler that ensures that the IRQ line has been deasserted
 * before returning.  Failing to do this will result in the IRQ
 * line being stuck active, and, since ISA irqs are edge triggered,
 * no more IRQs will be seen.
 */
static void serial_do_unlink(struct irq_info *i, struct uart_8250_port *up)
{
	spin_lock_irq(&i->lock);

	if (!list_empty(i->head)) {
		if (i->head == &up->list)
			i->head = i->head->next;
		list_del(&up->list);
	} else {
		BUG_ON(i->head != &up->list);
		i->head = NULL;
	}

	spin_unlock_irq(&i->lock);
}

static int serial_link_irq_chain(struct uart_8250_port *up)
{
	struct irq_info *i = irq_lists + up->port.irq;
	int ret, irq_flags = up->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;

	spin_lock_irq(&i->lock);

	if (i->head) {
		list_add(&up->list, i->head);
		spin_unlock_irq(&i->lock);

		ret = 0;
	} else {
		INIT_LIST_HEAD(&up->list);
		i->head = &up->list;
		spin_unlock_irq(&i->lock);

		ret = request_irq(up->port.irq, bbserial_interrupt,
				  irq_flags, "bbserial", i);
		if (ret < 0)
			serial_do_unlink(i, up);
	}

	return ret;
}

static void serial_unlink_irq_chain(struct uart_8250_port *up)
{
	struct irq_info *i = irq_lists + up->port.irq;

	BUG_ON(i->head == NULL);

	if (list_empty(i->head))
		free_irq(up->port.irq, i);

	serial_do_unlink(i, up);
}

/* Base timer interval for polling */
static inline int poll_timeout(int timeout)
{
	return timeout > 6 ? (timeout / 2 - 2) : 1;
}

/*
 * This function is used to handle ports that do not have an
 * interrupt.  This doesn't work very well for 16450's, but gives
 * barely passable results for a 16550A.  (Although at the expense
 * of much CPU overhead).
 */
static void bbserial_timeout(unsigned long data)
{
	struct uart_8250_port *up = (struct uart_8250_port *)data;
	unsigned int iir;

	iir = serial_in(up, UART_IIR);
	if (!(iir & UART_IIR_NO_INT))
		bbserial_handle_port(up);
	mod_timer(&up->timer, jiffies + poll_timeout(up->port.timeout));
}

static void bbserial_backup_timeout(unsigned long data)
{
	struct uart_8250_port *up = (struct uart_8250_port *)data;
	unsigned int iir, ier = 0, lsr;
	unsigned long flags;

	/*
	 * Must disable interrupts or else we risk racing with the interrupt
	 * based handler.
	 */
	if (is_real_interrupt(up->port.irq)) {
		ier = serial_in(up, UART_IER);
		serial_out(up, UART_IER, 0);
	}

	iir = serial_in(up, UART_IIR);

	/*
	 * This should be a safe test for anyone who doesn't trust the
	 * IIR bits on their UART, but it's specifically designed for
	 * the "Diva" UART used on the management processor on many HP
	 * ia64 and parisc boxes.
	 */
	spin_lock_irqsave(&up->port.lock, flags);
	lsr = serial_in(up, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	spin_unlock_irqrestore(&up->port.lock, flags);
	if ((iir & UART_IIR_NO_INT) && (up->ier & UART_IER_THRI) &&
	    (!uart_circ_empty(&up->port.info->xmit) || up->port.x_char) &&
	    (lsr & UART_LSR_THRE)) {
		iir &= ~(UART_IIR_ID | UART_IIR_NO_INT);
		iir |= UART_IIR_THRI;
	}

	if (!(iir & UART_IIR_NO_INT))
		bbserial_handle_port(up);

	if (is_real_interrupt(up->port.irq))
		serial_out(up, UART_IER, ier);

	/* Standard timer interval plus 0.2s to keep the port running */
	mod_timer(&up->timer, jiffies + poll_timeout(up->port.timeout) + HZ/5);
}

static unsigned int bbserial_tx_empty(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long flags;
	unsigned int lsr;
	
	spin_lock_irqsave(&up->port.lock, flags);
	lsr = serial_in(up, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return lsr & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
}

static unsigned int bbserial_get_mctrl(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned int status;
	unsigned int ret;
	
	status = check_modem_status(up);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void bbserial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned char mcr = 0;
	
	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;

	serial_out(up, UART_MCR, mcr);
}

static void bbserial_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long flags;
	
	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

/*
 *	Wait for transmitter & holding register to empty
 */
static inline void wait_for_xmitr(struct uart_8250_port *up, int bits)
{
	unsigned int status, tmout = 10000;
	
	/* Wait up to 10ms for the character(s) to be sent. */
	do {
		status = serial_in(up, UART_LSR);

		up->lsr_saved_flags |= status & LSR_SAVE_FLAGS;

		if (--tmout == 0)
			break;
		udelay(1);
	} while ((status & bits) != bits);

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		unsigned int tmout;
		for (tmout = 1000000; tmout; tmout--) {
			unsigned int msr = serial_in(up, UART_MSR);
			up->msr_saved_flags |= msr & MSR_SAVE_FLAGS;
			if (msr & UART_MSR_CTS)
				break;
			udelay(1);
			touch_nmi_watchdog();
		}
	}
}

static int bbserial_startup(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long flags;
	unsigned char lsr, iir;
	int retval;
	
	up->capabilities = uart_config[up->port.type].flags;
	up->mcr = 0;
	DEBUG_BB("%s entry\n", __FUNCTION__);

	if (up->port.type == PORT_16C950) {
		/* Wake up and initialize UART 
		 * @todo strange test, for bb based on 950 chip, 
		 * not enabled yet 
		 */
//		if ((info->ACR & 0x18) != 0x10) {
			up->acr = 0;
//		} else {
//			up->acr = 0x10;
//		}
		serial_outp(up, UART_LCR, 0xBF);
		serial_outp(up, UART_EFR, UART_EFR_ECB);
		serial_outp(up, UART_IER, 0);
		serial_outp(up, UART_LCR, 0);
		serial_icr_write(up, UART_CSR, 0); /* Reset the UART */
		serial_outp(up, UART_LCR, 0xBF);
		serial_outp(up, UART_EFR, UART_EFR_ECB);
		serial_outp(up, UART_LCR, 0);
	}

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	bbserial_clear_fifos(up);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_inp(up, UART_LSR);
	(void) serial_inp(up, UART_RX);
	(void) serial_inp(up, UART_IIR);
	(void) serial_inp(up, UART_MSR);

	/*
	 * At this point, there's no way the LSR could still be 0xff;
	 * if it is, then bail out, because there's likely no UART
	 * here.
	 */
	if (!(up->port.flags & UPF_BUGGY_UART) &&
	    (serial_inp(up, UART_LSR) == 0xff)) {
		printk("ttyB%d: LSR safety check engaged!\n", up->port.line);
		return -ENODEV;
	}

	if (is_real_interrupt(up->port.irq)) {
		/*
		 * Test for UARTs that do not reassert THRE when the
		 * transmitter is idle and the interrupt has already
		 * been cleared.  Real 16550s should always reassert
		 * this interrupt whenever the transmitter is idle and
		 * the interrupt is enabled.  Delays are necessary to
		 * allow register changes to become visible.
		 */
		spin_lock_irqsave(&up->port.lock, flags);

		wait_for_xmitr(up, UART_LSR_THRE);
		serial_out_sync(up, UART_IER, UART_IER_THRI);
		udelay(1); /* allow THRE to set */
		serial_in(up, UART_IIR);
		serial_out(up, UART_IER, 0);
		serial_out_sync(up, UART_IER, UART_IER_THRI);
		udelay(1); /* allow a working UART time to re-assert THRE */
		iir = serial_in(up, UART_IIR);
		serial_out(up, UART_IER, 0);

		spin_unlock_irqrestore(&up->port.lock, flags);

		/*
		 * If the interrupt is not reasserted, setup a timer to
		 * kick the UART on a regular basis.
		 */
		if (iir & UART_IIR_NO_INT) {
			pr_debug("ttyB%d - using backup timer\n", port->line);
			up->timer.function = bbserial_backup_timeout;
			up->timer.data = (unsigned long)up;
			mod_timer(&up->timer, jiffies +
			          poll_timeout(up->port.timeout) + HZ/5);
		}
	}

	/*
	 * If the "interrupt" for this port doesn't correspond with any
	 * hardware interrupt, we use a timer-based system.  The original
	 * driver used to do this with IRQ0.
	 */
	if (!is_real_interrupt(up->port.irq)) {
		up->timer.data = (unsigned long)up;
		mod_timer(&up->timer, jiffies + poll_timeout(up->port.timeout));
	} else {
		retval = serial_link_irq_chain(up);
		if (retval)
			return retval;
	}

	/*
	 * Now, initialize the UART
	 */
	serial_outp(up, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&up->port.lock, flags);
	/*	outb( UART_ACR, info->port+UART_SCR );
	outb( info->ACR, info->port+UART_ICR );
	 * 
	 * Most PC uarts need OUT2 raised to enable interrupts.
	 */
	if (is_real_interrupt(up->port.irq))
		up->port.mctrl |= TIOCM_OUT2;

	bbserial_set_mctrl(&up->port, up->port.mctrl);

	/*
	 * Do a quick test to see if we receive an
	 * interrupt when we enable the TX irq.
	 */
	serial_outp(up, UART_IER, UART_IER_THRI);
	lsr = serial_in(up, UART_LSR);
	iir = serial_in(up, UART_IIR);
	serial_outp(up, UART_IER, 0);

	if (lsr & UART_LSR_TEMT && iir & UART_IIR_NO_INT) {
		if (!(up->bugs & UART_BUG_TXEN)) {
			up->bugs |= UART_BUG_TXEN;
			pr_debug("ttyB%d - enabling bad tx status workarounds\n",
				 port->line);
		}
	} else {
		up->bugs &= ~UART_BUG_TXEN;
	}

	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Clear the interrupt registers again for luck, and clear the
	 * saved flags to avoid getting false values from polling
	 * routines or the previous session.
	 */
	serial_inp(up, UART_LSR);
	serial_inp(up, UART_RX);
	serial_inp(up, UART_IIR);
	serial_inp(up, UART_MSR);
	up->lsr_saved_flags = 0;
	up->msr_saved_flags = 0;

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_outp(up, UART_IER, up->ier);
	
	/*
	 * @todo added by old drv in port open, ¿?
	 */
//	DEBUG_BB("%s up->acr=0x%02X\n", __FUNCTION__, up->acr);
//	serial_out(up, UART_SCR, UART_ACR);
//	serial_out(up, UART_ICR, up->acr);
	
	return 0;
}

static void bbserial_shutdown(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long flags;

	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_outp(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl &= ~TIOCM_OUT2;

	bbserial_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_inp(up, UART_LCR) & ~UART_LCR_SBC);
	bbserial_clear_fifos(up);

	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	(void) serial_in(up, UART_RX);

	del_timer_sync(&up->timer);
	up->timer.function = bbserial_timeout;
	if (is_real_interrupt(up->port.irq))
		serial_unlink_irq_chain(up);
}

static unsigned int bbserial_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	/*
	 * Handle magic divisors for baud rates above baud_base on
	 * SMSC SuperIO chips.
	 */
	if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
	    baud == (port->uartclk/4))
		quot = 0x8001;
	else if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
		baud == (port->uartclk/8))
		quot = 0x8002;
	else
		quot = uart_get_divisor(port, baud);

	return quot;
}
static int bb_prechange_speed(struct uart_8250_port *up, unsigned int quot, unsigned int baud) {
	unsigned int unSC;
	unsigned int unDiv;
	unsigned int unPS;
	unsigned int unActualBaud;
	unsigned int unDelta;

	unsigned int unBestSC = 0;
	unsigned int unBestDiv = 0;
	unsigned int unBestPS = 0;
	unsigned int unBestActualBaud = 0;
	unsigned int unBestDelta = (unsigned int)-1;

	/* @todo hardcoded for PCI_DEVICE_ID_BB_1S_RS422_VEL_N */
	unsigned int unClock = 14745600;
	//unsigned int unClock = 14745600 / 8;
	
	/* @todo unClock table nedd to go in board startup, clock in uart_8250_port
	 * 
	if (info->state->type == PORT_16C950) {
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS422_15M) unClock = 60000000;
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS422_18M) unClock = 72000000;
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS232_PHOTON_1) unClock = 14745600 / 4;
		if (brd->dev->device == PCI_DEVICE_ID_BB_4S_RS232_PHOTON) unClock = 14745600 / 4;
		if (brd->dev->device == PCI_DEVICE_ID_BB_8S_RS232_PHOTON) unClock = 14745600 / 4;
	}

	if ((info->state->type >= PORT_BB16PCI958_16) && (info->state->type <= PORT_BB16PCI958_128)) {
		if (brd->dev->device == PCI_DEVICE_ID_BB_1S_RS422_VEL_N) unClock = 14745600;
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS422_VEL_N) unClock = 14745600;
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS422_VEL_OPTO_N) unClock = 14745600;
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS232_RS422_VEL_N) unClock = 14745600;
	}
	*/
	// Loop through the sample clock values :
	for ( unSC = 16 ; unSC >= 4 ; unSC = (up->port.type == PORT_16C950) ? unSC - 1 : unSC / 2 ) {
		// Loop through the prescaler values:
		for ( unPS = 8 ; unPS < 249 ; unPS++ ) {	
			unDiv = ( unClock * 8 ) / ( baud * unSC * unPS );
			if ((unDiv == 0) || (unDiv > 65535)) continue;
				unActualBaud = ( unClock * 8 ) / ( unDiv * unSC * unPS );
				unDelta = (unActualBaud > baud) ? unActualBaud - baud : baud - unActualBaud;

				if (unDelta < unBestDelta) {
					unBestPS = unPS;
					unBestSC = unSC;
					unBestDiv = unDiv;
					unBestActualBaud = unActualBaud;
					unBestDelta = unDelta;
				}
		}
	}

	if (unBestActualBaud != 0) {
		quot = (int)unBestDiv;
		DEBUG_BB("%s 950 uartclk %d, baud %d, Div %d, TCR 0x%02x, CPR 0x%02x\n",
			__FUNCTION__, up->port.uartclk, baud, quot, unBestSC, unBestPS);
		//bb_dump_scratch(up);
	}

	return quot;
}

static int bb_change_speed(struct uart_8250_port *up, unsigned int quot, unsigned int baud) {
	unsigned int unSC;
	unsigned int unDiv;
	unsigned int unPS;
	unsigned int unActualBaud;
	unsigned int unDelta;

	unsigned int unBestSC = 0;
	unsigned int unBestDiv = 0;
	unsigned int unBestPS = 0;
	unsigned int unBestActualBaud = 0;
	unsigned int unBestDelta = (unsigned int)-1;

	/* @todo hardcoded for PCI_DEVICE_ID_BB_1S_RS422_VEL_N */
	unsigned int unClock = 14745600;
	//unsigned int unClock = 14745600 / 8;
	
	/* @todo unClock table nedd to go in board startup, clock in uart_8250_port
	 * 
	if (info->state->type == PORT_16C950) {
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS422_15M) unClock = 60000000;
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS422_18M) unClock = 72000000;
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS232_PHOTON_1) unClock = 14745600 / 4;
		if (brd->dev->device == PCI_DEVICE_ID_BB_4S_RS232_PHOTON) unClock = 14745600 / 4;
		if (brd->dev->device == PCI_DEVICE_ID_BB_8S_RS232_PHOTON) unClock = 14745600 / 4;
	}

	if ((info->state->type >= PORT_BB16PCI958_16) && (info->state->type <= PORT_BB16PCI958_128)) {
		if (brd->dev->device == PCI_DEVICE_ID_BB_1S_RS422_VEL_N) unClock = 14745600;
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS422_VEL_N) unClock = 14745600;
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS422_VEL_OPTO_N) unClock = 14745600;
		if (brd->dev->device == PCI_DEVICE_ID_BB_2S_RS232_RS422_VEL_N) unClock = 14745600;
	}
	*/
	// Loop through the sample clock values :
	for ( unSC = 16 ; unSC >= 4 ; unSC = (up->port.type == PORT_16C950) ? unSC - 1 : unSC / 2 ) {
		// Loop through the prescaler values:
		for ( unPS = 8 ; unPS < 249 ; unPS++ ) {	
			unDiv = ( unClock * 8 ) / ( baud * unSC * unPS );
			if ((unDiv == 0) || (unDiv > 65535)) continue;
				unActualBaud = ( unClock * 8 ) / ( unDiv * unSC * unPS );
				unDelta = (unActualBaud > baud) ? unActualBaud - baud : baud - unActualBaud;

				if (unDelta < unBestDelta) {
					unBestPS = unPS;
					unBestSC = unSC;
					unBestDiv = unDiv;
					unBestActualBaud = unActualBaud;
					unBestDelta = unDelta;
				}
		}
	}

	if (unBestActualBaud != 0) {
		/* @ harcoded for PCI_DEVICE_ID_BB_1S_RS422_VEL_N 
		if (info->state->type == PORT_16C950) {
			info->MCR |= 0x80;
			serial_outp( info, UART_MCR, info->MCR);
			serial_icr_write( info, UART_TCR, (int)unBestSC );
			serial_icr_write( info, 1, (int)unBestPS );
		} else {
		*/
			serial_outp(up, UART_MSR, BRAINBOXES_SCR_BAUD_SC);
			serial_outp(up, UART_SCR, (int)unBestSC );
			serial_outp(up, UART_MSR, BRAINBOXES_SCR_BAUD_PS);
			serial_outp(up, UART_SCR, (int)unBestPS );
			serial_outp(up, UART_MSR, 0x00 );
		//}
		quot = (int)unBestDiv;
		DEBUG_BB( 
			"%s: 950 uartclk %d, baud %d, Div %d, TCR 0x%02x, CPR 0x%02x\n",
			__FUNCTION__, up->port.uartclk, baud, quot, unBestSC, unBestPS);
		//bb_dump_scratch(up);
	}

	return quot;
}

static void
bbserial_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
		
#ifdef CMSPAR 
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = bbserial_get_divisor(port, baud);

	/*
	 * Oxford Semi 952 rev B workaround
	 */
	if (up->bugs & UART_BUG_QUOT && (quot & 0xff) == 0)
		quot ++;

	if (up->capabilities & UART_CAP_FIFO && up->port.fifosize > 1) {
		if (baud < 2400)
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
		else
			fcr = uart_config[up->port.type].fcr;
	}

	/*
	 * MCR-based auto flow control.  When AFE is enabled, RTS will be
	 * deasserted when the receive FIFO contains more characters than
	 * the trigger, or the MCR RTS bit is cleared.  In the case where
	 * the remote UART is not using CTS auto flow control, we must
	 * have sufficient FIFO entries for the latency of the remote
	 * UART to respond.  IOW, at least 32 bytes of FIFO.
	 */
	if (up->capabilities & UART_CAP_AFE && up->port.fifosize >= 32) {
		up->mcr &= ~UART_MCR_AFE;
		if (termios->c_cflag & CRTSCTS)
			up->mcr |= UART_MCR_AFE;
	}

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);
	
	/* @todo is the correct place?
	 * BrainBoxes specific code to change speed
	 */
	//DEBUG_BB("%s quot: %ud  baud: %ud\n", __FUNCTION__, quot, baud);
	quot = bb_change_speed(up, quot, baud);
	
	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (!(up->bugs & UART_BUG_NOMSR) &&
			UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;
	if (up->capabilities & UART_CAP_UUE)
		up->ier |= UART_IER_UUE | UART_IER_RTOIE;

	serial_out(up, UART_IER, up->ier);

	if (up->capabilities & UART_CAP_EFR) {
		unsigned char efr = 0;
		/*
		 * TI16C752/Startech hardware flow control.  FIXME:
		 * - TI16C752 requires control thresholds to be set.
		 * - UART_MCR_RTS is ineffective if auto-RTS mode is enabled.
		 */
		if (termios->c_cflag & CRTSCTS)
			efr |= UART_EFR_CTS;

		serial_outp(up, UART_LCR, 0xBF);
		serial_outp(up, UART_EFR, efr);
	}

	if (up->capabilities & UART_NATSEMI) {
		/* Switch to bank 2 not bank 1, to avoid resetting EXCR2 */
		serial_outp(up, UART_LCR, 0xe0);
	} else {
		serial_outp(up, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */
	}
	
	/* @todo remove¿?
	if (up->port.type == PORT_BB16PCI958) {
		up->mcr &= ~UART_MCR_AFE;
		if (termios->c_cflag & CRTSCTS)
			up->mcr |= UART_MCR_AFE;
	}*/
	serial_dl_write(up, quot);

	/* @todo from 16750
	 * LCR DLAB must be set to enable 64-byte FIFO mode. If the FCR
	 * is written without DLAB set, this mode will be disabled.
	 */
	//if (up->port.type == PORT_BB16PCI958)
	//	serial_outp(up, UART_FCR, fcr);
	
	serial_outp(up, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;					/* Save LCR */
	if (up->port.type != PORT_16750) {
		if (fcr & UART_FCR_ENABLE_FIFO) {
			/* emulated UARTs (Lucent Venus 167x) need two steps */
			serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
		}
		serial_outp(up, UART_FCR, fcr);		/* set fcr */
	}
	bbserial_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static void
bbserial_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	struct uart_8250_port *p = (struct uart_8250_port *)port;

	bbserial_set_sleep(p, state != 0);

	if (p->pm)
		p->pm(port, state, oldstate);
}

/*
 * Resource handling.
 */
static int bbserial_request_std_resource(struct uart_8250_port *up)
{
	unsigned int size = 8 << up->port.regshift;
	int ret = 0;

	switch (up->port.iotype) {
	case UPIO_AU:
		size = 0x100000;
		/* fall thru */
	case UPIO_TSI:
	case UPIO_MEM32:
	case UPIO_MEM:
	case UPIO_DWAPB:
		if (!up->port.mapbase)
			break;

		if (!request_mem_region(up->port.mapbase, size, "bbserial")) {
			ret = -EBUSY;
			break;
		}

		if (up->port.flags & UPF_IOREMAP) {
			up->port.membase = ioremap(up->port.mapbase, size);
			if (!up->port.membase) {
				release_mem_region(up->port.mapbase, size);
				ret = -ENOMEM;
			}
		}
		break;

	case UPIO_HUB6:
	case UPIO_PORT:
		if (!request_region(up->port.iobase, size, "bbserial"))
			ret = -EBUSY;
		break;
	}
	return ret;
}

static void bbserial_release_std_resource(struct uart_8250_port *up)
{
	unsigned int size = 8 << up->port.regshift;

	switch (up->port.iotype) {
	case UPIO_AU:
		size = 0x100000;
		/* fall thru */
	case UPIO_TSI:
	case UPIO_MEM32:
	case UPIO_MEM:
	case UPIO_DWAPB:
		if (!up->port.mapbase)
			break;

		if (up->port.flags & UPF_IOREMAP) {
			iounmap(up->port.membase);
			up->port.membase = NULL;
		}

		release_mem_region(up->port.mapbase, size);
		break;

	case UPIO_HUB6:
	case UPIO_PORT:
		release_region(up->port.iobase, size);
		break;
	}
}

static void bbserial_release_port(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;

	bbserial_release_std_resource(up);
}

static int bbserial_request_port(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	int ret = 0;

	ret = bbserial_request_std_resource(up);

	return ret;
}

static void bbserial_config_port(struct uart_port *port, int flags)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	int probeflags = PROBE_ANY;
	int ret;

	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */
	ret = bbserial_request_std_resource(up);
	if (ret < 0)
		return;

	if (flags & UART_CONFIG_TYPE)
		autoconfig(up, probeflags);
	if (up->port.type != PORT_UNKNOWN && flags & UART_CONFIG_IRQ)
		autoconfig_irq(up);

	if (up->port.type == PORT_UNKNOWN)
		bbserial_release_std_resource(up);
}

static int
bbserial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->irq >= NR_IRQS || ser->irq < 0 ||
	    ser->baud_base < 9600 || ser->type < PORT_UNKNOWN ||
	    ser->type >= ARRAY_SIZE(uart_config) || ser->type == PORT_CIRRUS ||
	    ser->type == PORT_STARTECH)
		return -EINVAL;
	return 0;
}

static const char *
bbserial_type(struct uart_port *port)
{
	int type = port->type;

	if (type >= ARRAY_SIZE(uart_config))
		type = 0;
	return uart_config[type].name;
}

/*
 * Gets configuration for a bb port
 * @todo add stuff for change outside port space addres, this
 * 		 will be done via flags in uart_8250_port
 */
static void bbserial_get_config(struct uart_8250_port *up, 
	struct bbserial_port_config *conf) 
{
	unsigned char tmp;
	
	memset(conf, 0, sizeof(*conf));
	
	conf->afc_trigger_level = serial_scr_read(up, BRAINBOXES_SCR_AFC_TRIGGER);
	tmp = serial_scr_read(up, BRAINBOXES_SCR_CONF1);
	conf->duplex_mode = tmp & BRAINBOXES_SCR_CONF1_DUPLEX;
	conf->cts_true = tmp & BRAINBOXES_SCR_CONF1_CTSTRUE ? 1 : 0;
	
	/* @todo not info yet to support */
	conf->rx_trigger_level = 0;
	conf->soft_tx_limit = 0;
	conf->tx_trigger_level = 0;
}
/*
 * Configures a bb port
 * @todo add stuff for change outside port space addres, this
 * 		 will be done via flags in uart_8250_port
 */
static int bbserial_set_config(struct uart_8250_port *up, 
	struct bbserial_port_config *conf)
{
	unsigned char tmp;
	
	serial_scr_write(up, BRAINBOXES_SCR_AFC_TRIGGER, conf->afc_trigger_level);
	tmp = conf->duplex_mode & BRAINBOXES_SCR_CONF1_DUPLEX;
	tmp |= conf->cts_true ? BRAINBOXES_SCR_CONF1_CTSTRUE : 0x00;
	serial_scr_write(up, BRAINBOXES_SCR_CONF1, tmp);
	
	/* @todo not info yet to support
	 * conf->rx_trigger_level, conf->soft_tx_limit, conf->tx_trigger_level
	 */
	
	return 0;
}

static int bbserial_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg) {
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	struct bbserial_port_config __user *uconf = (void __user *)arg;
	struct bbserial_port_config tmp;
	int ret = -ENOIOCTLCMD;
	
	switch(cmd) {
	case BRAINBOXES_IOCSCONF:
		
		if (!(ret = copy_from_user(&tmp, uconf, sizeof(*uconf))) ) {
			ret = bbserial_set_config(up, &tmp);
		}
		DEBUG_BB("%s cmd: 0x%04X  write conf.\n", __FUNCTION__, cmd);
		break;
		
	case BRAINBOXES_IOCGCONF:
		bbserial_get_config(up, &tmp);
		ret = copy_to_user(uconf, &tmp, sizeof(*uconf));
	
		DEBUG_BB("%s Read conf. cmd: 0x%04X  ret: %d\n", __FUNCTION__, cmd, ret);
		break;
	}
	
	return ret;	
}

struct uart_ops bbserial_pops = {
	.tx_empty		= bbserial_tx_empty,
	.set_mctrl		= bbserial_set_mctrl,
	.get_mctrl		= bbserial_get_mctrl,
	.stop_tx		= bbserial_stop_tx,
	.start_tx		= bbserial_start_tx,
	.stop_rx		= bbserial_stop_rx,
	.enable_ms		= bbserial_enable_ms,
	.break_ctl		= bbserial_break_ctl,
	.startup		= bbserial_startup,
	.shutdown		= bbserial_shutdown,
	.set_termios	= bbserial_set_termios,
	.pm				= bbserial_pm,
	.type			= bbserial_type,
	.release_port	= bbserial_release_port,
	.request_port	= bbserial_request_port,
	.config_port	= bbserial_config_port,
	.ioctl 			= bbserial_ioctl,
};
/**
 *	serial8250_suspend_port - suspend one serial port
 *	@line:  serial line number
 *
 *	Suspend one serial port.
 */
 
void bbserial_suspend_port(int line)
{
	uart_suspend_port(&bbserial_reg, &bbserial_ports[line].port);
}

/**
 *	serial8250_resume_port - resume one serial port
 *	@line:  serial line number
 *
 *	Resume one serial port.
 */
void bbserial_resume_port(int line)
{
	struct uart_8250_port *up = &bbserial_ports[line];

	if (up->capabilities & UART_NATSEMI) {
		unsigned char tmp;

		/* Ensure it's still in high speed mode */
		serial_outp(up, UART_LCR, 0xE0);

		tmp = serial_in(up, 0x04); /* EXCR2 */
		tmp &= ~0xB0; /* Disable LOCK, mask out PRESL[01] */
		tmp |= 0x10;  /* 1.625 divisor for baud_base --> 921600 */
		serial_outp(up, 0x04, tmp);

		serial_outp(up, UART_LCR, 0);
	}
	uart_resume_port(&bbserial_reg, &up->port);
}

/*
 * @todo Change name, not register ports, only structure.
 */
void __init
bbserial_register_ports(struct uart_driver *drv, struct device *dev)
{
	int i;

	for (i = 0; i < nr_uarts; i++) {
		struct uart_8250_port *up = &bbserial_ports[i];
		
		up->port.line = i;
		spin_lock_init(&up->port.lock);

		init_timer(&up->timer);
		up->timer.function = bbserial_timeout;

		/*
		 * ALPHA_KLUDGE_MCR needs to be killed.
		 */
		up->mcr_mask = ~ALPHA_KLUDGE_MCR;
		up->mcr_force = ALPHA_KLUDGE_MCR;

		up->port.ops = &bbserial_pops;

		//up->port.dev = dev;
		//uart_add_one_port(drv, &up->port);
	}
}
