#include <lvgl.h>

// uncomment a library for display driver
#define USE_TFT_ESPI_LIBRARY
// #define USE_ARDUINO_GFX_LIBRARY

#include "lv_xiao_round_screen.h"
#include "lv_hardware_test.h"

void setup()
{
    Serial.begin( 115200 );  //prepare for possible serial debug 
    Serial.println( "XIAO round screen - LVGL_Arduino" );

    lv_init();

    lv_xiao_disp_init();
    lv_xiao_touch_init();

    lv_hardware_test();
}

void loop()
{
    lv_timer_handler();  //let the GUI do its work 
    delay( 5 );
}
