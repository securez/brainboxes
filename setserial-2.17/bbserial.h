/*
 *  linux/include/linux/bbserial.h
 *
 *  Copyright (C) 2004 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _LINUX_BBSERIAL_H
#define _LINUX_BBSERIAL_H
#include <linux/ioctl.h>
#include <linux/serial_core.h>

/*
 * config struct to set /get via ioctl
 */
struct bbserial_port_config {
	/*
	 * Duplex operation
	 *  0	-	RS485 half-duplex autogating mode
	 *  1	-	RS485 half-duplex Rx/Tx state set by RTS line
	 *  4	-	Tx and Rx both disabled
	 *  5	-	Tx enabled, Rx disabled (RS485 half-duplex manual)
	 *  6	-	Tx disabled, Rx enabled (RS485 half-duplex manual)
	 * 	7	-	full-duplex
	 */ 
	unsigned char duplex_mode;
	/*
	 * If 0, CTS reflects the state of the CTS line receiver.
	 * If 1, CTS is held TRUE
	 */
	unsigned char cts_true;
	/* @todo check
	 * Receive Trigger Level for 950 chips. (not yet implemented) Range 1-100
	 * As an extra feature, if both, RxTriggerLevel and TxTriggerLevel, are
	 * set to 0, the FIFOs are disabled. The default is set to 
	 * 100 by the driver.
	 */
	unsigned char rx_trigger_level;
	/* @todo check
	 * Transmit Trigger Level for 950 chips. (not yet implemented) Range 1-100,
	 * the default is set to 1 by the driver.
	 */
	unsigned char tx_trigger_level;
	/* @todo check
	 * The amount of characters transmitted on each TX empty interrupt
	 * in percent. The allowed range is 1 to 100. Setting it to 1 garanties
	 * that only 1 byte at a time is written to the FIFO. Setting it to 100
	 * garanties, that the FIFO is filled up to its maximum on transmit.
	 * The default, if the value is not present in Parameters, is 100 for all
	 * card with other than 750 or 950 chips. On 750 or 950 cards, the default
	 * is 24, to pass the HCT tests under standard configuration.
	 * As soon as the Parameters key for a card is present, the default changes
	 * to 100 automatically!!
	 */
	unsigned char soft_tx_limit;
	/* @todo check
	 * 
	 */
	unsigned char afc_trigger_level; 
}; /* @todo need for __attribute__ ((packed)); ?? */

/*
 * info struct to get via ioctl
 */
struct bbserial_port_info;

/*
 * ioctls
 */
#define BRAINBOXES_IOC_MAGIC	'k'
#define BRAINBOXES_IOCSCONF		_IOW(BRAINBOXES_IOC_MAGIC, 1, long)
#define BRAINBOXES_IOCGCONF		_IOR(BRAINBOXES_IOC_MAGIC, 2, long)
#define BRAINBOXES_IOCGINFO		_IOR(BRAINBOXES_IOC_MAGIC, 3, long)
#define BRAINBOXES_MAXNR 		3

/*
 * These are additional supported serial types.
 */
#define PORT_BB16PCI958		(PORT_MAX_8250 + 1)

#endif /* _LINUX_BBSERIAL_H */
