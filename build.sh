#!/usr/bin/bash

make

gcc -o imu_display imu_display.c -lncurses
