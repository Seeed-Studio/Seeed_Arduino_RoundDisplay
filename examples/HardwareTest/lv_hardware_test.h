
#include <SD.h>
#include "I2C_BM8563.h"

#define NUM_ADC_SAMPLE 20
#define RP2040_VREF 3300 // The actual voltage on 3V3 pin. (unit: mV)
static lv_obj_t *slider_label;
static lv_obj_t *battery_bar, *battery_label;

I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire);

int32_t battery_level_percent(void)
{
  int32_t mvolts = 0;
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
  for(int8_t i=0; i<NUM_ADC_SAMPLE; i++){
    mvolts += analogReadMilliVolts(D0);
  }
  mvolts /= NUM_ADC_SAMPLE;
#elif defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE) || defined(ARDUINO_SEEED_XIAO_NRF52840)
  int32_t adc_raw = 0;
  for(int8_t i=0; i<NUM_ADC_SAMPLE; i++){
    adc_raw += analogRead(D0);
  }
  adc_raw /= NUM_ADC_SAMPLE;
  mvolts = 2400 * adc_raw / (1<<12);
#elif defined(ARDUINO_SEEED_XIAO_RP2040)
  int32_t adc_raw = 0;
  for(int8_t i=0; i<NUM_ADC_SAMPLE; i++){
    adc_raw += analogRead(D0);
  }
  adc_raw /= NUM_ADC_SAMPLE;
  mvolts = RP2040_VREF * adc_raw / (1<<12);
#endif
  int32_t level = (mvolts - 1850) * 100 / 250; // 1850 ~ 2100
  level = (level<0) ? 0 : ((level>100) ? 100 : level); 
  return level;
}

void rtc_clock_cb(lv_timer_t * timer) {
    lv_obj_t *rtc_clock = (lv_obj_t *)timer->user_data;
    I2C_BM8563_TimeTypeDef tStruct;
    rtc.getTime(&tStruct);
    char rtc_time[10];
    sprintf(rtc_time, "%d:%d:%d", tStruct.hours, tStruct.minutes, tStruct.seconds);
    lv_label_set_text(rtc_clock, rtc_time);
}

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        LV_LOG_USER("State: %s\n", lv_obj_has_state(obj, LV_STATE_CHECKED) ? "On" : "Off");
    }
}

static void hardware_polled_cb(lv_timer_t * timer) {
    lv_obj_t *tf_state = (lv_obj_t *)timer->user_data;
    if(SD.begin(D2)){
      lv_obj_add_state(tf_state, LV_STATE_CHECKED);
      delay(200);
      SD.end();
    } else {
      lv_obj_clear_state(tf_state, LV_STATE_CHECKED); 
    }
    lv_label_set_text_fmt(battery_label, "%"LV_PRId32"%", battery_level_percent());
}

static void slider_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);
    lv_label_set_text_fmt(slider_label, "%"LV_PRId32, lv_slider_get_value(slider));
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);
}

void lv_hardware_test(void)
{
    lv_obj_t * slider = lv_slider_create(lv_scr_act());
    lv_obj_set_width(slider, 120);
    lv_obj_set_pos(slider, 60, 120);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    slider_label = lv_label_create(lv_scr_act());
    lv_label_set_text(slider_label, "0");
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);

    lv_obj_t * tf_label = lv_label_create(lv_scr_act());
    lv_label_set_text(tf_label, "tf-card");
    lv_obj_set_pos(tf_label, 90, 170);
    lv_obj_t * tf_state = lv_switch_create(lv_scr_act());
    lv_obj_set_pos(tf_state, 90, 190);
    lv_obj_add_state(tf_state, LV_STATE_CHECKED | LV_STATE_DISABLED);
    lv_obj_add_event_cb(tf_state, event_handler, LV_EVENT_ALL, NULL);
    lv_timer_create(hardware_polled_cb, 7000, tf_state);

    rtc.begin();
    I2C_BM8563_TimeTypeDef tStruct;
    rtc.getTime(&tStruct);
    char rtc_time[10];
    sprintf(rtc_time, "%d:%d:%d", tStruct.hours, tStruct.minutes, tStruct.seconds);
    lv_obj_t * rtc_clock = lv_label_create(lv_scr_act());
    lv_label_set_text(rtc_clock, rtc_time);
    lv_obj_set_pos(rtc_clock, 55, 45);
    lv_timer_create(rtc_clock_cb, 1000, rtc_clock);


    analogReadResolution(12);
#if defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE) || defined(ARDUINO_SEEED_XIAO_NRF52840)
    analogReference(AR_INTERNAL2V4); // 0.6V ref  1/4 Gain
#endif

    lv_obj_t* battery_outline = lv_obj_create(lv_scr_act());
    lv_obj_set_style_border_width(battery_outline, 2, 0);
    lv_obj_set_style_pad_all(battery_outline, 0, 0);
    lv_obj_set_style_radius(battery_outline, 8, 0);
    lv_obj_clear_flag(battery_outline, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(battery_outline, 40, 20);
    lv_obj_set_pos(battery_outline, 128, 42);

    battery_bar = lv_bar_create(battery_outline);
    lv_obj_set_size(battery_bar, 40, 20);
    lv_obj_align_to(battery_bar, battery_outline, LV_ALIGN_CENTER, 0, 0);
    lv_bar_set_range(battery_bar, 0, 100);
    lv_bar_set_value(battery_bar, 80, LV_ANIM_OFF);
    // lv_obj_set_style_bg_color(battery_bar, lv_palette_main(LV_PALETTE_GREEN), 0);

    battery_label = lv_label_create(battery_outline);
    lv_obj_align_to(battery_label, battery_outline, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text_fmt(battery_label, "80%");
}