#include "TFT_Clock.h"

void setup(void) {

    init_rtc();
    Serial.begin(115200);
    lv_xiao_touch_init();
    draw_clock();

    if (SD.begin(chipSelect)) {
        SDstatus = 1;
        return;
    }
    else{
        Serial.println("SD card not inserted or initialization failed!");
    }
}

void loop() {
    sdstatus();
    draw_clock_time();
    get_touch();
}
