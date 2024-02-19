obj-m += bmx160.o

all: module dt
	echo Builded Device Tree Overlay and kernel module

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

dt: bmx160_overlay.dts
	dtc -@ -I dts -O dtb -o bmx160_overlay.dtbo bmx160_overlay.dts

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf bmx160_overlay.dtbo
