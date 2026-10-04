KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

load:
	sudo insmod motor_hub_driver.ko
	sudo chmod 666 /dev/motor_hub

unload:
	sudo rmmod motor_hub_driver
