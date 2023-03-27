/* display and touch driver for XIAO round screen */

#include <Arduino.h>
#include <lvgl.h>
#include <SPI.h>
#include <Wire.h>

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define LVGL_BUFF_SIZE 10 // Number of rows

#define CHSC6X_I2C_ID 0x2e
#define CHSC6X_MAX_POINTS_NUM 1
#define CHSC6X_READ_POINT_LEN 5
#define TOUCH_INT D7

#ifndef XIAO_BL
#define XIAO_BL D6
#endif
#define XIAO_DC D3
#define XIAO_CS D1

uint8_t screen_rotation;

/* display driver : use TFT_eSPI or Arduino_GFX */
#if defined(USE_TFT_ESPI_LIBRARY) && defined(USE_ARDUINO_GFX_LIBRARY)
#error "More than one graphics library is defined."
#elif defined(USE_TFT_ESPI_LIBRARY)
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);
#elif defined(USE_ARDUINO_GFX_LIBRARY)
#include <Arduino_GFX_Library.h>
#define SPI_FREQ 20000000 // SPI bus frequency: 20Mhz
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
Arduino_DataBus *bus = new Arduino_ESP32SPI(XIAO_DC, XIAO_CS, SCK, MOSI, MISO, FSPI);
#elif defined(ARDUINO_Seeed_XIAO_nRF52840_Sense) || defined(ARDUINO_Seeed_XIAO_nRF52840)
Arduino_DataBus *bus = new Arduino_NRFXSPI(XIAO_DC, XIAO_CS, SCK, MOSI, MISO);
#elif defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE) || defined(ARDUINO_SEEED_XIAO_NRF52840)
Arduino_DataBus *bus = new Arduino_mbedSPI(XIAO_DC, XIAO_CS);
#elif defined(ARDUINO_SEEED_XIAO_RP2040)
Arduino_DataBus *bus = new Arduino_RPiPicoSPI(XIAO_DC, XIAO_CS, SCK, MOSI, MISO, spi0);
#else
Arduino_DataBus *bus = new Arduino_HWSPI(XIAO_DC, XIAO_CS);
#endif
Arduino_GFX *gfx = new Arduino_GC9A01(bus, -1, screen_rotation, true);
#else
#error "Please define a graphics library for display."
#endif

void xiao_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );
#if defined(USE_TFT_ESPI_LIBRARY)
    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();
#elif defined(USE_ARDUINO_GFX_LIBRARY)
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
    lv_disp_flush_ready( disp );
}

void xiao_disp_init(void)
{
#if XIAO_BL > 0
    pinMode(XIAO_BL, OUTPUT); //Turn on screen backlight
    digitalWrite(XIAO_BL, HIGH);
#endif

#if defined(USE_TFT_ESPI_LIBRARY)
    tft.begin();
    tft.setRotation( screen_rotation );
    tft.fillScreen(TFT_BLACK);
#elif defined(USE_ARDUINO_GFX_LIBRARY)
    gfx->begin(SPI_FREQ);
    gfx->fillScreen(BLACK);
#endif
}

void lv_xiao_disp_init(void) 
{
    xiao_disp_init();

    /*Initialize the display buffer*/
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t buf[ SCREEN_WIDTH * LVGL_BUFF_SIZE ];
    lv_disp_draw_buf_init( &draw_buf, buf, NULL, SCREEN_WIDTH * LVGL_BUFF_SIZE );

    /*Initialize the display driver for lvgl*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = xiao_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );
}



/* touch driver : chsc6x */

bool chsc6x_is_pressed(void)
{
    if(digitalRead(TOUCH_INT) != LOW) {
        delay(3);
        if(digitalRead(TOUCH_INT) != LOW)
        return false;
    }
    return true;
}

void chsc6x_convert_xy(uint8_t *x, uint8_t *y)
{
    uint8_t x_tmp = *x, y_tmp = *y, _end = 0;
    for(int i=1; i<=screen_rotation; i++){
        x_tmp = *x;
        y_tmp = *y;
        _end = (i % 2) ? SCREEN_WIDTH : SCREEN_HEIGHT;
        *x = y_tmp;
        *y = _end - x_tmp;
    }
}

void chsc6x_get_xy(lv_coord_t * x, lv_coord_t * y)
{
    uint8_t temp[CHSC6X_READ_POINT_LEN] = {0};
    uint8_t read_len = Wire.requestFrom(CHSC6X_I2C_ID, CHSC6X_READ_POINT_LEN);
    if(read_len == CHSC6X_READ_POINT_LEN){
        Wire.readBytes(temp, read_len);
        if (temp[0] == 0x01) {
        chsc6x_convert_xy(&temp[2], &temp[4]);
        *x = temp[2];
        *y = temp[4];
        }
    }
}

void chsc6x_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    lv_coord_t touchX, touchY;
    if( !chsc6x_is_pressed() )
    {
        data->state = LV_INDEV_STATE_REL;
    } else {
        data->state = LV_INDEV_STATE_PR;
        chsc6x_get_xy(&touchX, &touchY);
        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;
    }
}

void lv_xiao_touch_init(void)
{
    pinMode(TOUCH_INT, INPUT_PULLUP);
    Wire.begin(); // Turn on the IIC bus for touch driver
    /*Initialize the touch driver for lvgl*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = chsc6x_read;
    lv_indev_drv_register(&indev_drv);
}
