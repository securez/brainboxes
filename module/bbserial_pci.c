/*
 *  linux/driver/serial/bbserial_pci.c
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/bbserial.h>
#include <linux/bitops.h>

#include <asm/byteorder.h>
#include <asm/io.h>

#include "bbserial.h"

/*
 * Debug stuff
 */
#ifdef SERIAL_DEBUG_PCI
#undef SERIAL_DEBUG_PCI
#endif

#if 0
#define DEBUG_BB(fmt...)	printk(fmt)
#else
#define DEBUG_BB(fmt...)	do { } while (0)
#endif

/*
 * Configuration:
 *   share_irqs - whether we pass IRQF_SHARED to request_irq().  This option
 *                is unsafe when used on edge-triggered interrupts.
 */
static unsigned int share_irqs = SERIAL8250_SHARE_IRQS;

unsigned int nr_uarts = CONFIG_BBSERIAL_NR_UARTS;
struct uart_8250_port bbserial_ports[UART_NR];

struct irq_info irq_lists[NR_IRQS];

struct uart_driver bbserial_reg = {
	.owner			= THIS_MODULE,
	.driver_name	= "bbserial",
	.dev_name		= "ttyB",
	.major			= 0,
	.minor			= 0,
	.nr				= UART_NR,
//	.cons			= SERIAL8250_CONSOLE,
};

/*
 * init function returns:
 *  > 0 - number of ports
 *  = 0 - use board->num_ports
 *  < 0 - error
 */
struct pci_serial_quirk {
	u32	vendor;
	u32	device;
	u32	subvendor;
	u32	subdevice;
	int	(*init)(struct pci_dev *dev);
	int	(*setup)(struct serial_private *, struct bbserial_board *,
			 struct uart_port *, int);
	void	(*exit)(struct pci_dev *dev);
};

static void moan_device(const char *str, struct pci_dev *dev)
{
	printk(KERN_WARNING "%s: %s\n"
	       KERN_WARNING "Please send the output of lspci -vv, this\n"
	       KERN_WARNING "message (0x%04x,0x%04x,0x%04x,0x%04x), the\n"
	       KERN_WARNING "manufacturer and name of serial board or\n"
	       KERN_WARNING "modem board to rmk+serial@arm.linux.org.uk.\n",
	       pci_name(dev), str, dev->vendor, dev->device,
	       dev->subsystem_vendor, dev->subsystem_device);
}

static int
setup_port(struct serial_private *priv, struct uart_port *port,
	   int bar, int offset, int regshift)
{
	struct pci_dev *dev = priv->dev;
	unsigned long base, len;

	if (bar >= PCI_NUM_BAR_RESOURCES)
		return -EINVAL;

	base = pci_resource_start(dev, bar);

	if (pci_resource_flags(dev, bar) & IORESOURCE_MEM) {
		len =  pci_resource_len(dev, bar);

		if (!priv->remapped_bar[bar])
			priv->remapped_bar[bar] = ioremap(base, len);
		if (!priv->remapped_bar[bar])
			return -ENOMEM;

		port->iotype = UPIO_MEM;
		port->iobase = 0;
		port->mapbase = base + offset;
		port->membase = priv->remapped_bar[bar] + offset;
		port->regshift = regshift;
	} else {
		port->iotype = UPIO_PORT;
		port->iobase = base + offset;
		port->mapbase = 0;
		port->membase = NULL;
		port->regshift = 0;
	}
	return 0;
}

static int
pci_default_setup(struct serial_private *priv, struct bbserial_board *board,
		  struct uart_port *port, int idx)
{
	unsigned int bar, offset = board->first_offset, maxnr;
	
	bar = FL_GET_BASE(board->flags);
	if (board->flags & FL_BASE_BARS)
		bar += idx;
	else
		offset += idx * board->uart_offset;

	maxnr = (pci_resource_len(priv->dev, bar) - board->first_offset) >>
		(board->reg_shift + 3);

	if (board->flags & FL_REGION_SZ_CAP && idx >= maxnr)
		return 1;
				
	return setup_port(priv, port, bar, offset, board->reg_shift);
}

/*
 * Master list of serial port init/setup/exit quirks.
 * This does not describe the general nature of the port.
 * (ie, baud base, number and location of ports, etc)
 *
 * This list is ordered alphabetically by vendor then device.
 * Specific entries must come before more generic entries.
 */
static struct pci_serial_quirk pci_serial_quirks[] = {
	/*
	 * Brain Boxes cards
	 */
	{
		.vendor		= PCI_VENDOR_ID_BRAIN_BOXES,
		.device		= PCI_DEVICE_ID_BB_1S_RS422_VEL_N,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,
	}
};

static inline int quirk_id_matches(u32 quirk_id, u32 dev_id)
{
	return quirk_id == PCI_ANY_ID || quirk_id == dev_id;
}

static struct pci_serial_quirk *find_quirk(struct pci_dev *dev)
{
	struct pci_serial_quirk *quirk;

	for (quirk = pci_serial_quirks; ; quirk++)
		if (quirk_id_matches(quirk->vendor, dev->vendor) &&
		    quirk_id_matches(quirk->device, dev->device) &&
		    quirk_id_matches(quirk->subvendor, dev->subsystem_vendor) &&
		    quirk_id_matches(quirk->subdevice, dev->subsystem_device))
		 	break;
	return quirk;
}

static inline int get_pci_irq(struct pci_dev *dev,
				struct bbserial_board *board)
{
	if (board->flags & FL_NOIRQ)
		return 0;
	else
		return dev->irq;
}

/*
 * This is the configuration table for all of the PCI serial boards
 * which we support.  It is directly indexed by the pci_board_num_t enum
 * value, which is encoded in the pci_device_id PCI probe table's
 * driver_data member.
 *
 * The makeup of these names are:
 *  pbn_bn{_bt}_n_baud{_offsetinhex}
 *
 *  bn		= PCI BAR number
 *  bt		= Index using PCI BARs
 *  n		= number of serial ports
 *  baud	= baud rate
 *  offsetinhex	= offset for each sequential port (in hex)
 *
 * This table is sorted by (in order): bn, bt, baud, offsetindex, n.
 *
 * Please note: in theory if n = 1, _bt infix should make no difference.
 * ie, pbn_b0_1_115200 is the same as pbn_b0_bt_1_115200
 */
enum pci_board_num_t {
	pbn_default = 0,
	
	pbn_bb_1S_RS422_VEL,
	/*
	 * @todo support other BrainBoxes ports
	pbn_bb_1S_RS232,
	pbn_bb_2S_RS232_VEL,
	pbn_bb_2S_RS422_15M,
	pbn_bb_2S_RS232_PHOTON,
	pbn_bb_2S_RS422_18M,
	pbn_bb_2S_1P_RS232,
	pbn_bb_2S_2P_RS232,
	pbn_bb_2S_RS232,
	pbn_bb_3S_RS232,
	pbn_bb_4S_RS232_PHOTON,
	pbn_bb_4S_RS232,
	pbn_bb_4S_RS422_VEL,
	pbn_bb_8S_RS232,
	pbn_bb_8S_RS232_PHOTON,
	*/
};

/*
 * uart_offset - the space between channels
 * reg_shift   - describes how the UART registers are mapped
 *               to PCI memory by the card.
*/

static struct bbserial_board pci_boards[] __devinitdata = {
	[pbn_default] = {
		.flags			= FL_BASE0,
		.num_ports		= 1,
		.base_baud		= 115200,
		.uart_offset	= 8,
	},
	[pbn_bb_1S_RS422_VEL] = {
		.flags			= FL_BASE2,
		.num_ports		= 1,
		.base_baud		= 921600,
		.uart_offset	= 8,
	}
};

/*
 * Given a complete unknown PCI device, try to use some heuristics to
 * guess what the configuration might be, based on the pitiful PCI
 * serial specs.  Returns 0 on success, 1 on failure.
 */
static int __devinit
serial_pci_guess_board(struct pci_dev *dev, struct bbserial_board *board)
{
	int num_iomem, num_port, first_port = -1, i;
	
	/*
	 * If it is not a communications device or the programming
	 * interface is greater than 6, give up.
	 *
	 * (Should we try to make guesses for multiport serial devices
	 * later?) 
	 * @todo remove?
	 */
	if ((((dev->class >> 8) != PCI_CLASS_COMMUNICATION_SERIAL) &&
	     ((dev->class >> 8) != PCI_CLASS_COMMUNICATION_MODEM)) ||
	    (dev->class & 0xff) > 6) {
	    	DEBUG_BB("%s Not communication class(0x%X).\n", __FUNCTION__, dev->class);
		//return -ENODEV;
	}

	/*
	 * Check for iomem regions
	 */
	num_iomem = num_port = 0;
	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (pci_resource_flags(dev, i) & IORESOURCE_IO) {
			num_port++;
			if (first_port == -1)
				first_port = i;
		}
		if (pci_resource_flags(dev, i) & IORESOURCE_MEM)
			num_iomem++;
	}

	/*
	 * If there is 1 or 0 iomem regions, and exactly one port,
	 * use it.  We guess the number of ports based on the IO
	 * region size.
	 */
	if (num_iomem <= 1 && num_port == 1) {
		board->flags = first_port;
		board->num_ports = pci_resource_len(dev, first_port) / 8;
		DEBUG_BB("%s Guessed.\n", __FUNCTION__);
		return 0;
	}

	/*
	 * Now guess if we've got a board which indexes by BARs.
	 * Each IO BAR should be 8 bytes, and they should follow
	 * consecutively.
	 */
	first_port = -1;
	num_port = 0;
	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (pci_resource_flags(dev, i) & IORESOURCE_IO &&
		    pci_resource_len(dev, i) == 8 &&
		    (first_port == -1 || (first_port + num_port) == i)) {
			num_port++;
			if (first_port == -1)
				first_port = i;
		}
	}

	if (num_port > 1) {
		board->flags = first_port | FL_BASE_BARS;
		board->num_ports = num_port;
		DEBUG_BB("%s # Ports: %d\n", __FUNCTION__, num_port);
		return 0;
	}

	return -ENODEV;
}

static inline int
serial_pci_matches(struct bbserial_board *board,
		   struct bbserial_board *guessed)
{
	return
	    board->num_ports == guessed->num_ports &&
	    board->base_baud == guessed->base_baud &&
	    board->uart_offset == guessed->uart_offset &&
	    board->reg_shift == guessed->reg_shift &&
	    board->first_offset == guessed->first_offset;
}

struct serial_private *
bbserial_init_ports(struct pci_dev *dev, struct bbserial_board *board)
{
	struct uart_port serial_port;
	struct serial_private *priv;
	struct pci_serial_quirk *quirk;
	int rc, nr_ports, i;

	nr_ports = board->num_ports;
	DEBUG_BB("%s # ports: %d\n", __FUNCTION__, nr_ports);

	/*
	 * Find an init and setup quirks.
	 */
	quirk = find_quirk(dev);

	/*
	 * Run the new-style initialization function.
	 * The initialization function returns:
	 *  <0  - error
	 *   0  - use board->num_ports
	 *  >0  - number of ports
	 */
	if (quirk->init) {
		rc = quirk->init(dev);
		if (rc < 0) {
			priv = ERR_PTR(rc);
			goto err_out;
		}
		if (rc)
			nr_ports = rc;
	}

	priv = kzalloc(sizeof(struct serial_private) +
		       sizeof(unsigned int) * nr_ports,
		       GFP_KERNEL);
	if (!priv) {
		priv = ERR_PTR(-ENOMEM);
		goto err_deinit;
	}

	priv->dev = dev;
	priv->quirk = quirk;

	memset(&serial_port, 0, sizeof(struct uart_port));
	serial_port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
	serial_port.uartclk = board->base_baud * 16;
	serial_port.irq = get_pci_irq(dev, board);
	serial_port.dev = &dev->dev;
	
	for (i = 0; i < nr_ports; i++) {
		if (quirk->setup(priv, board, &serial_port, i))
			break;

#ifdef SERIAL_DEBUG_PCI
		printk("Setup PCI port: port %x, irq %d, type %d\n",
		       serial_port.iobase, serial_port.irq, serial_port.iotype);
#endif
		
		priv->line[i] = bbserial_register_port(&serial_port);
		if (priv->line[i] < 0) {
			printk(KERN_WARNING "Couldn't register serial port %s: %d\n", pci_name(dev), priv->line[i]);
			break;
		}
	}

	priv->nr = i;

	return priv;

 err_deinit:
	if (quirk->exit)
		quirk->exit(dev);
 err_out:
	return priv;
}

void bbserial_remove_ports(struct serial_private *priv)
{
	struct pci_serial_quirk *quirk;
	int i;

	DEBUG_BB("%s # removing ports: %d\n", __FUNCTION__, priv->nr);
	for (i = 0; i < priv->nr; i++)
		bbserial_unregister_port(priv->line[i]);

	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (priv->remapped_bar[i])
			iounmap(priv->remapped_bar[i]);
		priv->remapped_bar[i] = NULL;
	}

	/*
	 * Find the exit quirks.
	 */
	quirk = find_quirk(priv->dev);
	DEBUG_BB("%s # call exit quirk\n", __FUNCTION__);
	if (quirk->exit)
		quirk->exit(priv->dev);

	kfree(priv);
}

void bbserial_suspend_ports(struct serial_private *priv)
{
	int i;

	for (i = 0; i < priv->nr; i++)
		if (priv->line[i] >= 0)
			bbserial_suspend_port(priv->line[i]);
}

void bbserial_resume_ports(struct serial_private *priv)
{
	int i;

	/*
	 * Ensure that the board is correctly configured.
	 */
	if (priv->quirk->init)
		priv->quirk->init(priv->dev);

	for (i = 0; i < priv->nr; i++)
		if (priv->line[i] >= 0)
			bbserial_resume_port(priv->line[i]);
}

static struct uart_8250_port *bbserial_find_match_or_unused(struct uart_port *port)
{
	int i;

	/*
	 * First, find a port entry which matches.
	 */ 
	for (i = 0; i < nr_uarts; i++) {
		if (uart_match_port(&bbserial_ports[i].port, port)) {
			DEBUG_BB("%s uart_match_port: %d\n", __FUNCTION__, i);
			return &bbserial_ports[i];
		}
	}

	/*
	 * We didn't find a matching entry, so look for the first
	 * free entry.  We look for one which hasn't been previously
	 * used (indicated by zero iobase).
	 */
	for (i = 0; i < nr_uarts; i++) {
		if (bbserial_ports[i].port.type == PORT_UNKNOWN &&
		    bbserial_ports[i].port.iobase == 0) {
		    	
			DEBUG_BB("%s free entry: %d\n", __FUNCTION__, i);
			return &bbserial_ports[i];
		}
	}

	/*
	 * That also failed.  Last resort is to find any entry which
	 * doesn't have a real port associated with it.
	 */
	for (i = 0; i < nr_uarts; i++) {
		if (bbserial_ports[i].port.type == PORT_UNKNOWN) {
			DEBUG_BB("%s unknown port: %d\n", __FUNCTION__, i);
			return &bbserial_ports[i];
		}
	}

	return NULL;
}

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
/* @todo remove
static int __devinit bbserial_probe(struct platform_device *dev)
{
	struct plat_bbserial_port *p = dev->dev.platform_data;
	struct uart_port port;
	int ret, i;

	memset(&port, 0, sizeof(struct uart_port));

	for (i = 0; p && p->flags != 0; p++, i++) {
		port.iobase	= p->iobase;
		port.membase	= p->membase;
		port.irq	= p->irq;
		port.uartclk	= p->uartclk;
		port.regshift	= p->regshift;
		port.iotype	= p->iotype;
		port.flags	= p->flags;
		port.mapbase	= p->mapbase;
		port.hub6	= p->hub6;
		port.dev	= &dev->dev;
		if (share_irqs)
			port.flags |= UPF_SHARE_IRQ;
		ret = bbserial_register_port(&port);
		if (ret < 0) {
			dev_err(&dev->dev, "unable to register port at index %d "
				"(IO%lx MEM%llx IRQ%d): %d\n", i,
				p->iobase, (unsigned long long)p->mapbase,
				p->irq, ret);
		}
	}
	return 0;
}
*/
/*
 * Remove serial ports registered against a platform device.
 */
/* @todo remove
static int __devexit bbserial_remove(struct platform_device *dev)
{
	int i;

	for (i = 0; i < nr_uarts; i++) {
		struct uart_8250_port *up = &bbserial_ports[i];

		if (up->port.dev == &dev->dev)
			bbserial_unregister_port(i);
	}
	return 0;
}

static int bbserial_suspend(struct platform_device *dev, pm_message_t state)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = &bbserial_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			uart_suspend_port(&bbserial_reg, &up->port);
	}

	return 0;
}

static int bbserial_resume(struct platform_device *dev)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = &bbserial_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			bbserial_resume_port(i);
	}

	return 0;
}

static struct platform_driver bbserial_driver = {
	.probe		= bbserial_probe,
	.remove		= __devexit_p(bbserial_remove),
	.suspend	= bbserial_suspend,
	.resume		= bbserial_resume,
	.driver		= {
	.name	= "bbserial",
	.owner	= THIS_MODULE,
	},
};
*/
/*
 * This "device" covers _all_ ISA 8250-compatible serial devices listed
 * in the table in include/asm/serial.h
 * @todo remove
 */
static struct platform_device *bbserial_devs;

/*
 * bbserial_register_port and bbserial_unregister_port allows for
 * 16x50 serial ports to be configured at run-time, to support PCMCIA
 * modems and PCI multiport cards.
 */
static DEFINE_MUTEX(serial_mutex);

/**
 *	bbserial_register_port - register a serial port
 *	@port: serial port template
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use, it is hung up and unregistered
 *	first.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
int bbserial_register_port(struct uart_port *port)
{
	struct uart_8250_port *uart;
	int ret = -ENOSPC;

	if (port->uartclk == 0)
		return -EINVAL;

	mutex_lock(&serial_mutex);

	uart = bbserial_find_match_or_unused(port);
	if (uart) {
		//uart_remove_one_port(&bbserial_reg, &uart->port);

		uart->port.iobase   = port->iobase;
		uart->port.membase  = port->membase;
		uart->port.irq      = port->irq;
		uart->port.uartclk  = port->uartclk;
		uart->port.fifosize = port->fifosize;
		uart->port.regshift = port->regshift;
		uart->port.iotype   = port->iotype;
		uart->port.flags    = port->flags | UPF_BOOT_AUTOCONF;
		uart->port.mapbase  = port->mapbase;
		if (port->dev)
			uart->port.dev = port->dev;

		ret = uart_add_one_port(&bbserial_reg, &uart->port);
		if (ret == 0)
			ret = uart->port.line;
	}
	mutex_unlock(&serial_mutex);

	return ret;
}

/**
 *	bbserial_unregister_port - remove a 16x50 serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may not be called from interrupt
 *	context.  We hand the port back to the our control.
 */
void bbserial_unregister_port(int line)
{
	struct uart_8250_port *uart = &bbserial_ports[line];

	DEBUG_BB("%s entry line: %d\n", __FUNCTION__, line);

	mutex_lock(&serial_mutex);
	uart_remove_one_port(&bbserial_reg, &uart->port);
	if (bbserial_devs) {
		DEBUG_BB("%s # bbserial_devs exists, reregister port\n", __FUNCTION__);
		uart->port.flags &= ~UPF_BOOT_AUTOCONF;
		uart->port.type = PORT_UNKNOWN;
		uart->port.dev = &bbserial_devs->dev;
		uart_add_one_port(&bbserial_reg, &uart->port);
	} else {
		DEBUG_BB("%s # bbserial_devs not exists\n", __FUNCTION__);
		uart->port.iobase = 0;
		uart->port.dev = NULL;
	}
	mutex_unlock(&serial_mutex);
}

/*
 * Probe one serial board.  Unfortunately, there is no rhyme nor reason
 * to the arrangement of serial ports on a PCI card.
 */
static int __devinit
bbserial_init_one(struct pci_dev *dev, const struct pci_device_id *ent)
{
	struct serial_private *priv;
	struct bbserial_board *board, tmp;
	int rc;

	DEBUG_BB("%s entry\n", __FUNCTION__);

	if (ent->driver_data >= ARRAY_SIZE(pci_boards)) {
		printk(KERN_ERR "pci_init_one: invalid driver_data: %ld\n",
			ent->driver_data);
		return -EINVAL;
	}

	board = &pci_boards[ent->driver_data];

	DEBUG_BB("%s enabling device\n", __FUNCTION__);
	rc = pci_enable_device(dev);
	if (rc)
		return rc;

	if (ent->driver_data == pbn_default) {
		DEBUG_BB("%s default driver data\n", __FUNCTION__);
		/*
		 * Use a copy of the pci_board entry for this;
		 * avoid changing entries in the table.
		 */
		memcpy(&tmp, board, sizeof(struct bbserial_board));
		board = &tmp;

		/*
		 * We matched one of our class entries.  Try to
		 * determine the parameters of this board.
		 */
		rc = serial_pci_guess_board(dev, board);
		if (rc)
			goto disable;
	} else {
		DEBUG_BB("%s mached explicit entry\n", __FUNCTION__);
		/*
		 * We matched an explicit entry.  If we are able to
		 * detect this boards settings with our heuristic,
		 * then we no longer need this entry.
		 */
		memcpy(&tmp, &pci_boards[pbn_default],
		       sizeof(struct bbserial_board));
		rc = serial_pci_guess_board(dev, &tmp);
		if (rc == 0 && serial_pci_matches(board, &tmp))
			moan_device("Redundant entry in serial pci_table.",
				    dev);
	}

	DEBUG_BB("%s Init ports\n", __FUNCTION__);
	priv = bbserial_init_ports(dev, board);
	if (!IS_ERR(priv)) {
		pci_set_drvdata(dev, priv);	
		
		/*
	 	 * @todo remove debug here
	 	 *
		DEBUG_BB("%s PCI Resources:\n", __FUNCTION__);
		for(i = 0; i < PCI_NUM_BAR_RESOURCES; i++)
			DEBUG_BB("%s  Num: %d  Start: 0x%lX  Len: %lu  Type: %s\n", 
				__FUNCTION__, i, pci_resource_start(dev, i),
				pci_resource_len(dev, i),
				((pci_resource_flags(dev, i) & IORESOURCE_MEM) ? "MEM" : "PORT")
				);
		*/
		return 0;
	}

	rc = PTR_ERR(priv);

 disable:
	pci_disable_device(dev);
	return rc;
}

static void __devexit bbserial_remove_one(struct pci_dev *dev)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	pci_set_drvdata(dev, NULL);
	DEBUG_BB("%s Removing ports.\n", __FUNCTION__);
	bbserial_remove_ports(priv);

	pci_disable_device(dev);
	
}

#ifdef CONFIG_PM
static int bbserial_suspend_one(struct pci_dev *dev, pm_message_t state)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	if (priv)
		bbserial_suspend_ports(priv);

	pci_save_state(dev);
	pci_set_power_state(dev, pci_choose_state(dev, state));
	return 0;
}

static int bbserial_resume_one(struct pci_dev *dev)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	pci_set_power_state(dev, PCI_D0);
	pci_restore_state(dev);

	if (priv) {
		/*
		 * The device may have been disabled.  Re-enable it.
		 */
		pci_enable_device(dev);

		bbserial_resume_ports(priv);
	}
	return 0;
}
#endif

static struct pci_device_id bbserial_tbl[] = {
	{	PCI_VENDOR_ID_BRAIN_BOXES, PCI_DEVICE_ID_BB_1S_RS422_VEL_N,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, 
		pbn_bb_1S_RS422_VEL },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, bbserial_tbl);

static struct pci_driver serial_pci_driver = {
	.name		= "bbserial",
	.probe		= bbserial_init_one,
	.remove		= __devexit_p(bbserial_remove_one),
#ifdef CONFIG_PM
	.suspend	= bbserial_suspend_one,
	.resume		= bbserial_resume_one,
#endif
	.id_table	= bbserial_tbl,
};

static int __init bbserial_pci_init(void)
{
	int rc, i;
	
	/*
	 * Init the UART driver
	 */
	if (nr_uarts > UART_NR)
		nr_uarts = UART_NR;
		
	// @todo remove
	printk("%s, nr ports:%d\n", __FUNCTION__, nr_uarts);

	printk(KERN_INFO "Serial: BrainBoxes driver $Revision: 1.00 $ "
		"%d ports, IRQ sharing %sabled\n", nr_uarts,
		share_irqs ? "en" : "dis");
	

	for (i = 0; i < NR_IRQS; i++)
		spin_lock_init(&irq_lists[i].lock);

	rc = uart_register_driver(&bbserial_reg);
	if (rc)
		goto out;

	bbserial_devs = NULL;
// @todo remove
//	bbserial_devs = platform_device_alloc("bbserial",
//						    PLAT8250_DEV_LEGACY);
//	if (!bbserial_devs) {
//		rc = -ENOMEM;
//		goto unreg_uart_drv;
//	}

//	rc = platform_device_add(bbserial_devs);
//	if (rc)
//		goto put_dev;

	bbserial_register_ports(&bbserial_reg, &bbserial_devs->dev);

//	rc = platform_driver_register(&bbserial_driver);
//	if (rc != 0)
//		goto del_dev;
		
	return pci_register_driver(&serial_pci_driver);	

// del_dev:
//	platform_device_del(bbserial_devs);
// put_dev:
// 	DEBUG_BB("%s remove platform dev device\n", __FUNCTION__);
//	platform_device_put(bbserial_devs);
// unreg_uart_drv:
 	DEBUG_BB("%s unregister driver driver\n", __FUNCTION__);
	uart_unregister_driver(&bbserial_reg);
 out:
	return rc;
}

static void __exit bbserial_pci_exit(void)
{
	//struct platform_device *bb_dev = bbserial_devs;
	
	//DEBUG_BB("%s Unregister PCI driver\n", __FUNCTION__);
	pci_unregister_driver(&serial_pci_driver);
	
	/*
	 * This tells bbserial_unregister_port() not to re-register
	 * the ports (thereby making bbserial_isa_driver permanently
	 * in use.)
	 */
	DEBUG_BB("%s Unregister UART driver\n", __FUNCTION__);
	bbserial_devs = NULL;

	//platform_driver_unregister(&bbserial_driver);
	//platform_device_unregister(bb_dev);

	uart_unregister_driver(&bbserial_reg);
}

module_init(bbserial_pci_init);
module_exit(bbserial_pci_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Brain Boxes Serial PCI");
MODULE_AUTHOR("Marcos Lois <marcos.lois@gmail.com>");

module_param(share_irqs, uint, 0644);
MODULE_PARM_DESC(share_irqs, "Share IRQs with other non-8250/16x50 devices"
	" (unsafe)");

module_param(nr_uarts, uint, 0644);
MODULE_PARM_DESC(nr_uarts, "Maximum number of UARTs supported. (1-" __MODULE_STRING(CONFIG_SERIAL_8250_NR_UARTS) ")");

MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);
