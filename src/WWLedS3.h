#pragma once

#include <stdint.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_io_i80.h"

#include "soc/soc_memory_types.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "esp_pm.h"

#include "soc/soc_memory_types.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "esp_pm.h"
//#include "esp_lcd_panel_io_interface.h"
//#include "esp_lcd_panel_io.h"
#include "esp_rom_gpio.h"
#include "soc/soc_caps.h"
#include "soc/rtc.h" // for `rtc_clk_xtal_freq_get()`
#include "soc/soc_memory_types.h"
#include "hal/dma_types.h"
#include "hal/gpio_hal.h"
#include "esp_private/gdma.h"
#include "driver/gpio.h"
//#include "esp_private/periph_ctrl.h"

//#include "esp_lcd_common.h"
#include "soc/lcd_periph.h"
#include "hal/lcd_ll.h"
#include "hal/lcd_hal.h"

#include "esp_log.h"
#include "soc/gdma_reg.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_ops.h"
#include "esp_timer.h"

#define LCD_DRIVER_PSRAM_DATA_ALIGNMENT 64
// #define CLOCKLESS_PIXEL_CLOCK_HZ  (24 * 100 * 1000)


#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define __OFFSET 0 //  (24*3*2*2*2+2)
#define __OFFSET_END  (24*3*2*2*2+2)

#ifdef COLOR_ORDER_GRBW
#define _p_r 1
#define _p_g 0
#define _p_b 2
#define _nb_components 4
#else
#ifdef COLOR_ORDER_RGB
#define _p_r 0
#define _p_g 1
#define _p_b 2
#define _nb_components 3
#else
#ifdef  COLOR_ORDER_RBG
#define _p_r 0
#define _p_g 2
#define _p_b 1
#define _nb_components 3
#else
#ifdef COLOR_ORDER_GBR
#define _p_r 2
#define _p_g 0
#define _p_b 1
#define _nb_components 3
#else
#ifdef COLOR_ORDER_BGR
#define _p_r 2
#define _p_g 1
#define _p_b 0
#define _nb_components 3
#else
#ifdef COLOR_ORDER_BRG
#define _p_r 1
#define _p_g 2
#define _p_b 0
#define _nb_components 3
#else
#ifdef COLOR_ORDER_GRB
#define _p_r 1
#define _p_g 0
#define _p_b 2
#define _nb_components 3
#else

#define _p_r 1
#define _p_g 0
#define _p_b 2
#define _nb_components 3
#endif
#endif
#endif
#endif
#endif
#endif
#endif

//------------------------------
// Unified color type
//------------------------------
typedef union {
    struct __attribute__((packed)) {
        uint8_t r, g, b, w;
    };
    uint32_t num;
} color_t;

//------------------------------
// LED configuration structure
//------------------------------
typedef struct {
    int num_strips;           // Number of LED strips (max 16)
    int num_leds_per_strip;   // LEDs per strip
    int nb_components;        // 3 for RGB, 4 for RGBW
    const int* pins;          // Array of pin numbers (length >= num_strips)
    uint8_t brightness;       // 0-255 brightness setting
    float gammaR;             // Gamma for red channel
    float gammaG;             // Gamma for green channel
    float gammaB;             // Gamma for blue channel
    float gammaW;             // Gamma for white channel (if applicable)
    color_t* pixel_buffer;    // Pointer to pixel data; must be allocated by caller
} led_config_t;

//------------------------------
// LED driver class for ESP32-S3
//------------------------------
class WWLedS3 {
public:
    WWLedS3();
    ~WWLedS3();

    // Initialize the driver from a configuration structure.
    // The configuration struct will typically be populated from JSON.
    void init(const led_config_t* config);

    // Set brightness and update internal gamma mapping tables.
    void setBrightness(uint8_t brightness);

    // Set gamma correction values.
    void setGamma(float gammaR, float gammaG, float gammaB);
    void setGamma(float gammaR, float gammaG, float gammaB, float gammaW);

    // Set the color for a specific LED using a color_t.
    // The pixel ordering is assumed to be: for each strip, all its pixels are stored contiguously.
    // (i.e., pixel (i, j) is at index: i * num_leds_per_strip + j)
    void setPixelColor(int strip, int pixel, const color_t& color);

    // Convenience functions to construct a color_t from individual channels.
    static color_t setRGB(uint8_t r, uint8_t g, uint8_t b);
    static color_t setRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

    // Transmit the current pixel data to update the LEDs.
    void show();

    // Return a pointer to the pixel buffer.
    color_t* getPixelBuffer() { return pixels; }

private:
    // Configuration variables (set via init())
    int num_strips;
    int num_leds_per_strip;
    int nb_components;    // 3 or 4
    const int* pins;      // Pointer to an array of pin numbers
    uint8_t brightness;
    float gammaR, gammaG, gammaB, gammaW;
    color_t* pixels;      // Pointer to the user-supplied pixel buffer

    // Gamma mapping arrays.
    uint8_t redMap[256];
    uint8_t greenMap[256];
    uint8_t blueMap[256];
    uint8_t whiteMap[256];

    // Hardware interface
    esp_lcd_panel_io_handle_t led_io_handle;  // Obtained via esp_lcd_new_panel_io_i80()
    uint16_t* buffers[2];  // Double-buffering for transmission.
    uint16_t* ledOutput;
    uint16_t* ledOutput2;
    int currentFrame;

    // Semaphore for synchronizing transfers.
    SemaphoreHandle_t transferSemaphore;

    // Debug counters and flags.
    int testCount;
    bool isDisplaying;
    bool isWaiting;

    // Constants for buffer allocation and clock configuration.
    static const int OFFSET = 0;
    static const int OFFSET_END = (24 * 3 * 2 * 2 * 2 + 2);
    static const int CLOCKLESS_PIXEL_CLOCK_HZ = (24 * 100 * 1000);

    // Internal helper functions.
    // Configure and initialize the LCD-I80 bus hardware.
    void initHardware();

    // Prepare (transpose) the pixel data into the bit-packed format required for transmission.
    void transposeAll(uint16_t* ledoutput);

    // Helper: bit-transpose 16 bytes into 24 16-bit words.
    static void IRAM_ATTR transpose16x1_noinline2(uint8_t* A, uint16_t* B);

    // Callback invoked upon transmission completion.
    static bool IRAM_ATTR flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                      esp_lcd_panel_io_event_data_t* edata,
                                      void* user_ctx);
};

