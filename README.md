# Seeed Studio Round Display for XIAO
## Introduction 

An Arduino graphics library based on LVGL, specifically designed for Round Display for XIAO.

## Depends

    lvgl, lv_examples, TFT_eSPI, GFX Library for Arduino, I2C BM8563 RTC, SD, AnimatedGIF

## Usage

    1.Git clone this resp to your Arduino IDE's libraries directory.
    2.Install all the dependencies from the Arduino Library Manager.
    3.Uncomment a library for display driver.
    4.Run the demo "HardTest" on examples directory.

## Note

Please install the dependencies from the Arduino Library Manager. 

If you want to use the TFT_eSPI library for display driving, you must comment out the line `#include <User_Setup.h>` and uncomment the line  `#include <User_Setups/Setup66_Seeed_XIAO_RoundDisplay.h> ` in the "User_Setup_Select.h" file.

The lvgl configuration file `lv_conf.h`  must be copied to  your Arduino IDE's libraries directory.
