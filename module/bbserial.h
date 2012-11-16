/*
 *  linux/driver/serial/bbserial.h
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
 * @todo reorder thinks
 */
#include <linux/serial_core.h>
#include "include/linux/bbserial.h"

/* @todo complete list, contact brainboxes
 * Scrach Register numbers, used for extra functionality
 */
#define BRAINBOXES_SCR				0x00
#define BRAINBOXES_SCR_CHIPIDENT	0x02
#define BRAINBOXES_SCR_BAUD_SC		0x03
#define BRAINBOXES_SCR_UCR			0x06
	#define BRAINBOXES_SCR_UCR_MAXFIFO		0x02
#define BRAINBOXES_SCR_FIFO_SIZE	0x09
#define BRAINBOXES_SCR_CONF1		0x0A
	#define BRAINBOXES_SCR_CONF1_DUPLEX			0x07
	#define BRAINBOXES_SCR_CONF1_CTSTRUE		0x08
#define BRAINBOXES_SCR_AFC_TRIGGER	0x0E
#define BRAINBOXES_SCR_BAUD_PS		0x10


	



/*
 * @todo is ISA dependant strunture?
 */
struct old_serial_port {
	unsigned int uart;
	unsigned int baud_base;
	unsigned int port;
	unsigned int irq;
	unsigned int flags;
	unsigned char hub6;
	unsigned char io_type;
	unsigned char *iomem_base;
	unsigned short iomem_reg_shift;
};

/*
 * This replaces serial_uart_config in include/linux/serial.h
 */
struct bbserial_config {
	const char	*name;
	unsigned short	fifo_size;
	unsigned short	tx_loadsz;
	unsigned char	fcr;
	unsigned int	flags;
};

#define PCI_NUM_BAR_RESOURCES	6
struct serial_private {
	struct pci_dev		*dev;
	unsigned int		nr;
	void __iomem		*remapped_bar[PCI_NUM_BAR_RESOURCES];
	struct pci_serial_quirk	*quirk;
	int		line[0];
};

/*
 * Max uart number
 */
#define CONFIG_BBSERIAL_NR_UARTS 16
#define UART_NR	CONFIG_BBSERIAL_NR_UARTS 

/*
 * Definitions for PCI support.
 */
#define FL_BASE_MASK	0x0007
#define FL_BASE0		0x0000
#define FL_BASE1		0x0001
#define FL_BASE2		0x0002
#define FL_BASE3		0x0003
#define FL_BASE4		0x0004
#define FL_GET_BASE(x)	(x & FL_BASE_MASK)

/* Use successive BARs (PCI base address registers),
   else use offset into some specified BAR */
#define FL_BASE_BARS	0x0008

/* do not assign an irq */
#define FL_NOIRQ		0x0080

/* Use the Base address register size to cap number of ports */
#define FL_REGION_SZ_CAP	0x0100

/*
 * Per-board information
 */
struct bbserial_board {
	unsigned int flags;
	unsigned int num_ports;
	unsigned int base_baud;
	unsigned int uart_offset;
	unsigned int reg_shift;
	unsigned int first_offset;
};

struct irq_info {
	spinlock_t		lock;
	struct list_head	*head;
};

struct serial_private;

struct serial_private *
bbserial_init_ports(struct pci_dev *dev, struct bbserial_board *board);
void bbserial_remove_ports(struct serial_private *priv);
void bbserial_suspend_ports(struct serial_private *priv);
void bbserial_resume_ports(struct serial_private *priv);
void __init bbserial_register_ports(struct uart_driver *drv, struct device *dev);

/*
 * BB port
 */
struct uart_8250_port {
	struct uart_port	port;
	struct timer_list	timer;		/* "no irq" timer */
	struct list_head	list;		/* ports on this IRQ */
	unsigned short		capabilities;	/* port capabilities */
	unsigned short		bugs;		/* port bugs */
	unsigned int		tx_loadsz;	/* transmit fifo load size */
	unsigned char		acr;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */

	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
#define LSR_SAVE_FLAGS UART_LSR_BRK_ERROR_BITS
	unsigned char		lsr_saved_flags;
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
	unsigned char		msr_saved_flags;

	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};

/*
 * This is the platform device platform_data structure
 */
struct plat_bbserial_port {
	unsigned long	iobase;		/* io base address */
	void __iomem	*membase;	/* ioremap cookie or NULL */
	resource_size_t	mapbase;	/* resource base */
	unsigned int	irq;		/* interrupt number */
	unsigned int	uartclk;	/* UART clock rate */
	unsigned char	regshift;	/* register shift */
	unsigned char	iotype;		/* UPIO_* */
	unsigned char	hub6;
	upf_t		flags;			/* UPF_* flags */
};

/*
 * Allocate 8250 platform device IDs.  Nothing is implied by
 * the numbering here, except for the legacy entry being -1.
 * @todo search this
 */
enum {
	PLAT8250_DEV_LEGACY = -1,
	PLAT8250_DEV_PLATFORM,
	PLAT8250_DEV_PLATFORM1,
	PLAT8250_DEV_PLATFORM2,
	PLAT8250_DEV_FOURPORT,
	PLAT8250_DEV_ACCENT,
	PLAT8250_DEV_BOCA,
	PLAT8250_DEV_EXAR_ST16C554,
	PLAT8250_DEV_HUB6,
	PLAT8250_DEV_MCA,
	PLAT8250_DEV_AU1X00,
};

/*
 * This should be used by drivers which want to register
 * their own 8250 ports without registering their own
 * platform device.  Using these will make your driver
 * dependent on the 8250 driver.
 * @remove
 */

struct uart_port;

int bbserial_register_port(struct uart_port *);
void bbserial_unregister_port(int line);
void bbserial_suspend_port(int line);
void bbserial_resume_port(int line);

extern int early_serial_setup(struct uart_port *port);

int bbserial_find_port(struct uart_port *p);
int bbserial_find_port_for_earlycon(void);
int setup_early_bbserial_console(char *cmdline);



#define UART_CAP_FIFO	(1 << 8)	/* UART has FIFO */
#define UART_CAP_EFR	(1 << 9)	/* UART has EFR */
#define UART_CAP_SLEEP	(1 << 10)	/* UART has IER sleep */
#define UART_CAP_AFE	(1 << 11)	/* MCR-based hw flow control */
#define UART_CAP_UUE	(1 << 12)	/* UART needs IER bit 6 set (Xscale) */

#define UART_BUG_QUOT	(1 << 0)	/* UART has buggy quot LSB */
#define UART_BUG_TXEN	(1 << 1)	/* UART has buggy TX IIR status */
#define UART_BUG_NOMSR	(1 << 2)	/* UART has buggy MSR status bits (Au1x00) */

#define PROBE_RSA	(1 << 0)
#define PROBE_ANY	(~0)

#define HIGH_BITS_OFFSET ((sizeof(long)-sizeof(int))*8)

#ifdef CONFIG_SERIAL_8250_SHARE_IRQ
#define SERIAL8250_SHARE_IRQS 1
#else
#define SERIAL8250_SHARE_IRQS 0
#endif

#if defined(__alpha__) && !defined(CONFIG_PCI)
/*
 * Digital did something really horribly wrong with the OUT1 and OUT2
 * lines on at least some ALPHA's.  The failure mode is that if either
 * is cleared, the machine locks up with endless interrupts.
 */
#define ALPHA_KLUDGE_MCR  (UART_MCR_OUT2 | UART_MCR_OUT1)
#else
#define ALPHA_KLUDGE_MCR 0
#endif

#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

/*
 * Devinitions for device IDs
 * @todo move to pci_ids
 */
#define PCI_VENDOR_ID_BRAIN_BOXES				0x135a
#define PCI_DEVICE_ID_BB_1S_RS232				0x02a0	// tested - PnP ID should be 2a1!!
#define PCI_DEVICE_ID_BB_1S_RS232_N				0x0aa1	// *
#define PCI_DEVICE_ID_BB_1S_RS422_VEL_N			0x0a61	// *
#define PCI_DEVICE_ID_BB_2S_RS232_VEL_1			0x00e1	// tested
#define PCI_DEVICE_ID_BB_2S_RS422_VEL			0x00a1	// tested
#define PCI_DEVICE_ID_BB_2S_RS422_VEL_N			0x08a1	// *
#define PCI_DEVICE_ID_BB_2S_RS422_VEL_OPTO		0x00c2	// untested
#define PCI_DEVICE_ID_BB_2S_RS422_VEL_OPTO_N	0x08c1	// *
#define PCI_DEVICE_ID_BB_2S_RS232_RS422_VEL		0x0281	// tested
#define PCI_DEVICE_ID_BB_2S_RS232_RS422_VEL_N	0x0a81	// *
#define PCI_DEVICE_ID_BB_2S_RS422_15M			0x0201	// 15M is not a standard supported baudrate
#define PCI_DEVICE_ID_BB_2S_RS422_18M			0x0361	// 18M is not a standard supported baudrate
#define PCI_DEVICE_ID_BB_2S_RS232_PHOTON_1		0x01a1	// none in stock
#define PCI_DEVICE_ID_BB_2S_1P_RS232_1			0x0061	// tested
#define PCI_DEVICE_ID_BB_2S_1P_RS232_2			0x02e1	// tested
#define PCI_DEVICE_ID_BB_2S_1P_RS232_3			0x0181	// tested
#define PCI_DEVICE_ID_BB_2S_1P_RS232_N1			0x0861	// *
#define PCI_DEVICE_ID_BB_2S_1P_RS232_N2			0x0bc1	// untested
#define PCI_DEVICE_ID_BB_2S_1P_RS232_N3			0x0981	// *
#define PCI_DEVICE_ID_BB_2S_2P_RS232			0x01e1	// tested
#define PCI_DEVICE_ID_BB_2S_RS232_5V			0x02c1	// none in stock
#define PCI_DEVICE_ID_BB_2S_RS232_12V			0x0321	// none in stock
#define PCI_DEVICE_ID_BB_2S_RS232_LP			0x0ba1	// untested
#define PCI_DEVICE_ID_BB_2S_RS232_SC			0x0ca1	// untested
#define PCI_DEVICE_ID_BB_3S_RS232_1				0x0101	// untested
#define PCI_DEVICE_ID_BB_3S_RS232_2				0x0102	// tested
#define PCI_DEVICE_ID_BB_3S_RS232_N1			0x0901	// *
#define PCI_DEVICE_ID_BB_3S_RS232_N2			0x0501	// *
#define PCI_DEVICE_ID_BB_4S_RS232_PHOTON		0x01c4	// untested
#define PCI_DEVICE_ID_BB_4S_RS232_POS_1			0x0161	// none in stock
#define PCI_DEVICE_ID_BB_4S_RS232_1				0x042	// tested
#define PCI_DEVICE_ID_BB_4S_RS232_2				0x043	// untested
#define PCI_DEVICE_ID_BB_4S_RS232_3				0x0121	// none in stock
#define PCI_DEVICE_ID_BB_4S_RS232_N1			0x0521	// *
#define PCI_DEVICE_ID_BB_4S_RS232_N3			0x0841	// *
#define PCI_DEVICE_ID_BB_4S_RS232_N4			0x0921	// tested
#define PCI_DEVICE_ID_BB_4S_RS422_VEL			0x0301	// none in stock
#define PCI_DEVICE_ID_BB_4S_RS422_VEL_OPTO_1	0x0441	// untested
#define PCI_DEVICE_ID_BB_4S_RS422_VEL_OPTO_2	0x0442	// tested
#define PCI_DEVICE_ID_BB_4S_RS232_LP			0x0d21	// untested
#define PCI_DEVICE_ID_BB_4S_RS232_RJ45			0x0d41	// *
#define PCI_DEVICE_ID_BB_8S_RS232_3				0x0083	// tested
#define PCI_DEVICE_ID_BB_8S_RS232_N1			0x0881	// *
#define PCI_DEVICE_ID_BB_8S_RS232_PHOTON		0x0224	// untested
