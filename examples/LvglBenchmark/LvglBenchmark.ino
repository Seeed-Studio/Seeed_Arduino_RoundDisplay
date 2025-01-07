#include <lvgl.h>
#include <demos/lv_demos.h>
// uncomment a library for display driver
#define USE_TFT_ESPI_LIBRARY
// #define USE_ARDUINO_GFX_LIBRARY

#include "lv_xiao_round_screen.h"

void setup()
{
    Serial.begin( 115200 );  //prepare for possible serial debug 
    Serial.println( "XIAO round screen - LVGL_Arduino" );

    lv_init();

    #if LVGL_VERSION_MAJOR == 9
    lv_tick_set_cb(millis);
    #endif

    lv_xiao_disp_init();
    lv_xiao_touch_init();

    // lv_demo_widgets();
    lv_demo_benchmark();
    // lv_demo_keypad_encoder();
    // lv_demo_music();
    // lv_demo_printer();
    // lv_demo_stress();
}

void loop()
{
    lv_timer_handler();  //let the GUI do its work 
    delay( 5 );
}
