## BMX160 Linux Device Driver

### How to use
Clone the repository and type the following commands in the terminal
```
cd bmx160-linux-device_driver
bash build.sh
sudo dtoverlay bmx160_overlay.dtbo
sudo insmod bmx160.ko
./imu_display
```

![](https://github.com/Muhammed-Hamdan/bmx160-linux-device_driver/blob/main/bmx160_test.gif)

 ### How to remove module and device tree
 ```
sudo rmmod bmx160
sudo dtoverlay -r bmx160_overlay
 ```
