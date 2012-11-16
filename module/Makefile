# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

KERNELDIR=/usr/src/linux-source-3.2.0/linux-source-3.2.0

# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g  # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2
endif

EXTRA_CFLAGS += $(DEBFLAGS)
EXTRA_CFLAGS += -I$(PWD)/include

ifneq ($(KERNELRELEASE),)
# call from kernel build system

obj-m += bbserial.o
bbserial-objs := bbserial_pci.o bbserial_tty.o

else

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules V=1

endif

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

depend .depend dep:
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) -M *.c > .depend


ifeq (.depend,$(wildcard .depend))
include .depend
endif

#all:
#	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
#	
#clean:
#	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
#	rm -f *.symvers *~
