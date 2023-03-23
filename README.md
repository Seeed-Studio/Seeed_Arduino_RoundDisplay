# Seeed Studio Round Display for XIAO
## Introduction 

An Arduino graphics library based on LVGL, specifically designed for Round Display for XIAO.

## Usage

    1.Git clone this resp to your Arduino IDE's libraries directory.
    2.Run the demo "HardTest" on examples directory.
    3.Uncomment a library for display driver

## Note

If you want to use the TFT_eSPI library for display driving, you must comment out the line `#include <User_Setup.h>` and uncomment the line  `#include <User_Setups/Setup66_Seeed_XIAO_RoundDisplay.h> ` in the "User_Setup_Select.h" file.

The lvgl configuration file `lv_conf.h`  must be copied to  your Arduino IDE's libraries directory.
