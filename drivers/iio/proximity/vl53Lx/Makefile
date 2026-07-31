ifneq ($(KERNELRELEASE),)
include Kbuild

else
KDIR ?= /lib/modules/`uname -r`/build

default:
	CONFIG_STMVL53LX=m $(MAKE) -C $(KDIR) M=$$PWD
clean:
	CONFIG_STMVL53LX=m $(MAKE) -C $(KDIR) M=$$PWD clean

install:
	CONFIG_STMVL53LX=m $(MAKE) -C $(KDIR) M=$$PWD modules_install
endif
