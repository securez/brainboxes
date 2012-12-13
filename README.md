brainboxes
==========

This is a Linux 2.6.x driver for RS485 BrainBroxes PCI Card. The manufacturer also released a Linux driver but stop development at 2.4.25.

When us really need migrate customer server to 2.6.x, i decide to port the driver to 2.6.x.

The chip is based on standard pc serial chip 8250 with some obscure modifications, since it change startup process, and add some ioctl for board config, i fork the 8250 PCI driver, i wasn't confortable with mantain a set of patches that i can't mantain for futire versions of linux kernel, the manufacturer refuse send me any info about some obscure registers so i only enable the 2.4.x present features.

It's tested on Linux kernel up to 2.6.27, this is a old project, :P. before commit to github i tested on 2.3.0 and doesn't compile, but seems to be a renamed structure.

Actually the driver support this card:

    PCI_DEVICE_ID_BB_1S_RS422_VEL_N

But original 2.4.x driver supports:

    PCI_DEVICE_ID_BB_1S_RS232  	            0x2a0	// tested - PnP ID should be 2a1!!
    PCI_DEVICE_ID_BB_1S_RS232_N		        0xaa1	// *
    PCI_DEVICE_ID_BB_1S_RS422_VEL_N		    0xa61	// *
    PCI_DEVICE_ID_BB_2S_RS232_VEL_1		    0xe1	// tested
    PCI_DEVICE_ID_BB_2S_RS422_VEL		    0xa1	// tested
    PCI_DEVICE_ID_BB_2S_RS422_VEL_N		    0x8a1	// *
    PCI_DEVICE_ID_BB_2S_RS422_VEL_OPTO	    0xc2	// untested
    PCI_DEVICE_ID_BB_2S_RS422_VEL_OPTO_N	0x8c1	// *
    PCI_DEVICE_ID_BB_2S_RS232_RS422_VEL		0x281	// tested
    PCI_DEVICE_ID_BB_2S_RS232_RS422_VEL_N	0xa81	// *
    PCI_DEVICE_ID_BB_2S_RS422_15M		    0x201	// 15M is not a standard supported baudrate
    PCI_DEVICE_ID_BB_2S_RS422_18M		    0x361	// 18M is not a standard supported baudrate
    PCI_DEVICE_ID_BB_2S_RS232_PHOTON_1		0x1a1	// none in stock
    PCI_DEVICE_ID_BB_2S_1P_RS232_1		    0x61	// tested
    PCI_DEVICE_ID_BB_2S_1P_RS232_2		    0x2e1	// tested
    PCI_DEVICE_ID_BB_2S_1P_RS232_3		    0x181	// tested
    PCI_DEVICE_ID_BB_2S_1P_RS232_N1		    0x861	// *
    PCI_DEVICE_ID_BB_2S_1P_RS232_N2		    0xbc1	// untested
    PCI_DEVICE_ID_BB_2S_1P_RS232_N3		    0x981	// *
    PCI_DEVICE_ID_BB_2S_2P_RS232		    0x1e1	// tested
    PCI_DEVICE_ID_BB_2S_RS232_5V		    0x2c1	// none in stock
    PCI_DEVICE_ID_BB_2S_RS232_12V		    0x321	// none in stock
    PCI_DEVICE_ID_BB_2S_RS232_LP			0xba1	// untested
    PCI_DEVICE_ID_BB_2S_RS232_SC			0xca1	// untested
    PCI_DEVICE_ID_BB_3S_RS232_1		        0x101	// untested
    PCI_DEVICE_ID_BB_3S_RS232_2		        0x102	// tested
    PCI_DEVICE_ID_BB_3S_RS232_N1		    0x901	// *
    PCI_DEVICE_ID_BB_3S_RS232_N2		    0x501	// *
    PCI_DEVICE_ID_BB_4S_RS232_PHOTON		0x1c4	// untested
    PCI_DEVICE_ID_BB_4S_RS232_POS_1		    0x161	// none in stock
    PCI_DEVICE_ID_BB_4S_RS232_1		        0x42	// tested
    PCI_DEVICE_ID_BB_4S_RS232_2		        0x43	// untested
    PCI_DEVICE_ID_BB_4S_RS232_3		        0x121	// none in stock
    PCI_DEVICE_ID_BB_4S_RS232_N1		    0x521	// *
    PCI_DEVICE_ID_BB_4S_RS232_N3		    0x841	// *
    PCI_DEVICE_ID_BB_4S_RS232_N4		    0x921	// tested
    PCI_DEVICE_ID_BB_4S_RS422_VEL		    0x301	// none in stock
    PCI_DEVICE_ID_BB_4S_RS422_VEL_OPTO_1	0x441	// untested
    PCI_DEVICE_ID_BB_4S_RS422_VEL_OPTO_2	0x442	// tested
    PCI_DEVICE_ID_BB_4S_RS232_LP			0xd21	// untested
    PCI_DEVICE_ID_BB_4S_RS232_RJ45			0xd41	// *
    PCI_DEVICE_ID_BB_8S_RS232_3		        0x83	// tested
    PCI_DEVICE_ID_BB_8S_RS232_N1		    0x881	// *
    PCI_DEVICE_ID_BB_8S_RS232_PHOTON		0x224	// untested

So support any of these card, will be easy adding PCI id to table, and some minor fix.

The original 2.4.x drivers can be found at:

http://www.brainboxes.com/faq/items/where-can-i-find-the-latest-drivers-for-linux?file=files/pages/support/faqs/drivers/Serial%20solutions%20for%20Linux%201.0.2.zip
