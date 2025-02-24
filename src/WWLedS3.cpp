#include "WWLedS3.h"
#include <string.h>

// Macros used for the bit-transposition.
#define AA  (0x00AA00AAL)
#define CC  (0x0000CCCCL)
#define FF  (0xF0F0F0F0L)
#define FF2 (0x0F0F0F0FL)

// Logging tag.
static const char* TAG = "WWLedS3";

//------------------------------------------------------------
// Constructor: Initialize default parameters.
WWLedS3::WWLedS3()
    : num_strips(0),
      num_leds_per_strip(0),
      nb_components(3),
      pins(nullptr),
      brightness(255),
      gammaR(1.0f), gammaG(1.0f), gammaB(1.0f), gammaW(1.0f),
      pixels(nullptr),
      led_io_handle(nullptr),
      ledOutput(nullptr),
      ledOutput2(nullptr),
      currentFrame(0),
      transferSemaphore(nullptr),
      testCount(0),
      isDisplaying(false),
      isWaiting(false)
{
    // Initialize gamma maps to identity.
    for (int i = 0; i < 256; i++) {
        redMap[i]   = i;
        greenMap[i] = i;
        blueMap[i]  = i;
        whiteMap[i] = i;
    }
}

//------------------------------------------------------------
// Destructor: Free any allocated buffers.
WWLedS3::~WWLedS3() {
    if (ledOutput) {
        heap_caps_free(ledOutput - OFFSET/2); // Adjust pointer if necessary.
    }
    if (ledOutput2) {
        heap_caps_free(ledOutput2 - OFFSET/2);
    }
    // Deinitialization of led_io_handle can be added if required.
}

//------------------------------------------------------------
// Initialize the LED driver using the provided configuration.
void WWLedS3::init(const led_config_t* config) {
    num_strips         = config->num_strips;
    num_leds_per_strip = config->num_leds_per_strip;
    nb_components      = config->nb_components;
    pins               = config->pins;
    brightness         = config->brightness;
    gammaR             = config->gammaR;
    gammaG             = config->gammaG;
    gammaB             = config->gammaB;
    gammaW             = config->gammaW;
    pixels             = config->pixel_buffer;  // Caller must allocate a buffer of size: num_strips * num_leds_per_strip

    currentFrame = 0;
    setBrightness(brightness);

    // Create the semaphore if needed.
    if (transferSemaphore == nullptr) {
        transferSemaphore = xSemaphoreCreateBinary();
    }

    // Calculate the size for the internal output buffers.
    int buf_size = 8 * nb_components * num_leds_per_strip * 3 * 2 + OFFSET + OFFSET_END;
    ledOutput = (uint16_t*)heap_caps_aligned_alloc(LCD_DRIVER_PSRAM_DATA_ALIGNMENT, buf_size,
                                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ledOutput) {
        memset(ledOutput, 0, buf_size);
    }
    ledOutput2 = (uint16_t*)heap_caps_aligned_alloc(LCD_DRIVER_PSRAM_DATA_ALIGNMENT, buf_size,
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ledOutput2) {
        memset(ledOutput2, 0, buf_size);
    }
    buffers[0] = ledOutput;
    buffers[1] = ledOutput2;
    // Adjust pointers as in the original code.
    ledOutput  += OFFSET/2;
    ledOutput2 += OFFSET/2;

    // Pre-set a timing marker in the buffers (as in original code).
    for (int i = 0; i < num_leds_per_strip * nb_components * 8; i++) {
        ledOutput[3 * i + 1] = 0xFFFF;
        ledOutput2[3 * i + 1] = 0xFFFF;
    }

    // Initialize the hardware interface (LCD-I80 bus).
    initHardware();
}

//------------------------------------------------------------
// Set brightness and update gamma mapping tables.
void WWLedS3::setBrightness(uint8_t brightness) {
    this->brightness = brightness;
    float tmp;
    for (int i = 0; i < 256; i++) {
        tmp = powf((float)i / 255.0f, 1.0f / gammaG);
        greenMap[i] = (uint8_t)(tmp * brightness);
        tmp = powf((float)i / 255.0f, 1.0f / gammaR);
        redMap[i]   = (uint8_t)(tmp * brightness);
        tmp = powf((float)i / 255.0f, 1.0f / gammaB);
        blueMap[i]  = (uint8_t)(tmp * brightness);
        if (nb_components > 3) {
            tmp = powf((float)i / 255.0f, 1.0f / gammaW);
            whiteMap[i] = (uint8_t)(tmp * brightness);
        }
    }
}

//------------------------------------------------------------
// Set gamma correction (RGB version).
void WWLedS3::setGamma(float gammaR, float gammaG, float gammaB) {
    this->gammaR = gammaR;
    this->gammaG = gammaG;
    this->gammaB = gammaB;
    setBrightness(this->brightness);
}

//------------------------------------------------------------
// Set gamma correction (RGBW version).
void WWLedS3::setGamma(float gammaR, float gammaG, float gammaB, float gammaW) {
    this->gammaR = gammaR;
    this->gammaG = gammaG;
    this->gammaB = gammaB;
    this->gammaW = gammaW;
    setBrightness(this->brightness);
}

//------------------------------------------------------------
// Set the color for a specific LED using a color_t value.
// Pixel ordering: pixels for each strip are stored contiguously.
// Index = (strip index * num_leds_per_strip) + pixel index.
void WWLedS3::setPixelColor(int strip, int pixel, const color_t& color) {
    int index = strip * num_leds_per_strip + pixel;
    pixels[index] = color;
}

//------------------------------------------------------------
// Convenience function to construct an RGB color.
color_t WWLedS3::setRGB(uint8_t r, uint8_t g, uint8_t b) {
    color_t col;
    col.r = r;
    col.g = g;
    col.b = b;
    col.w = 0;
    return col;
}

//------------------------------------------------------------
// Convenience function to construct an RGBW color.
color_t WWLedS3::setRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    color_t col;
    col.r = r;
    col.g = g;
    col.b = b;
    col.w = w;
    return col;
}

//------------------------------------------------------------
// The show() method prepares the current frame by transposing pixel data,
// waits for any previous transmission to finish, and triggers transmission.
void WWLedS3::show() {
    // Prepare the current frame buffer.
    transposeAll(buffers[currentFrame]);

    // If a transmission is in progress, wait.
    if (isDisplaying) {
        isWaiting = true;
        if (transferSemaphore == nullptr) {
            transferSemaphore = xSemaphoreCreateBinary();
        }
        xSemaphoreTake(transferSemaphore, portMAX_DELAY);
    }
    isDisplaying = true;
    size_t tx_size = nb_components * num_leds_per_strip * 8 * 3 * 2 + OFFSET + OFFSET_END;
    led_io_handle->tx_color(led_io_handle, 0x2C, buffers[currentFrame], tx_size);
    currentFrame = (currentFrame + 1) % 2;
}

//------------------------------------------------------------
// Internal: Initialize the LCD-I80 bus (used here as the I2S-like interface).
void WWLedS3::initHardware() {
    esp_lcd_i80_bus_handle_t i80_bus = nullptr;
    esp_lcd_i80_bus_config_t bus_config = {};
    bus_config.clk_src = LCD_CLK_SRC_PLL160M;
    bus_config.dc_gpio_num = 0;
    bus_config.wr_gpio_num = 0;
    // Copy the configured pin numbers.
    for (int i = 0; i < num_strips; i++) {
        bus_config.data_gpio_nums[i] = pins[i];
    }
    // For unused data lines (up to 16), set them to 0.
    for (int i = num_strips; i < 16; i++) {
        bus_config.data_gpio_nums[i] = 0;
    }
    bus_config.bus_width = 16;
    bus_config.max_transfer_bytes = nb_components * num_leds_per_strip * 8 * 3 * 2 + OFFSET;
    bus_config.psram_trans_align = LCD_DRIVER_PSRAM_DATA_ALIGNMENT;
    bus_config.sram_trans_align = 4;
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    esp_lcd_panel_io_i80_config_t io_config = {};
    io_config.cs_gpio_num = -1;
    io_config.pclk_hz = CLOCKLESS_PIXEL_CLOCK_HZ;
    io_config.trans_queue_depth = 1;
    io_config.dc_levels = {
        .dc_idle_level = 0,
        .dc_cmd_level = 0,
        .dc_dummy_level = 0,
        .dc_data_level = 1,
    };
    io_config.lcd_cmd_bits = 0;
    io_config.lcd_param_bits = 0;
    io_config.user_ctx = this;
    io_config.on_color_trans_done = flush_ready;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &led_io_handle));
}

//------------------------------------------------------------
// Internal: Transpose the pixel data into the bit-packed output format.
// Assumes that pixels are stored per strip contiguously; for each pixel index,
// the pixel from strip i is at index = (i * num_leds_per_strip) + j.
void WWLedS3::transposeAll(uint16_t* ledoutput) {
    uint16_t ledToDisplay = 0;
    // Temporary storage for up to 4 channels.
    union {
        uint8_t bytes[16];
        uint32_t d[4];
    } secondPixel[4];  // Use 4 channels for RGBW.

    // The pointer into the output buffer (skip initial words as in original).
    uint16_t* buff = ledoutput + 2;
    for (int j = 0; j < num_leds_per_strip; j++) {
        // For each strip, get the corresponding pixel (from the user buffer).
        for (int i = 0; i < num_strips; i++) {
            // Calculate index assuming each strip's data is contiguous.
            int index = i * num_leds_per_strip + j;
            color_t col = pixels[index];
            secondPixel[0].bytes[i] = redMap[col.r];
            secondPixel[1].bytes[i] = greenMap[col.g];
            secondPixel[2].bytes[i] = blueMap[col.b];
            if (nb_components > 3) {
                secondPixel[3].bytes[i] = whiteMap[col.w];
            }
        }
        ledToDisplay++;
        transpose16x1_noinline2(secondPixel[0].bytes, buff); buff += 24;
        transpose16x1_noinline2(secondPixel[1].bytes, buff); buff += 24;
        transpose16x1_noinline2(secondPixel[2].bytes, buff); buff += 24;
        if (nb_components > 3) {
            transpose16x1_noinline2(secondPixel[3].bytes, buff); buff += 24;
        }
    }
}

//------------------------------------------------------------
// Internal: Bit-transpose 16 bytes into 24 16-bit words.
// Marked IRAM_ATTR so that it can safely be called from an ISR.
void IRAM_ATTR WWLedS3::transpose16x1_noinline2(uint8_t* A, uint16_t* B) {
    uint32_t x, y, x1, y1, t;
    y = *(uint32_t *)(A);
#if (NUMSTRIPS > 4)
    x = *(uint32_t *)(A + 4);
#else
    x = 0;
#endif
#if (NUMSTRIPS > 8)
    y1 = *(uint32_t *)(A + 8);
#else
    y1 = 0;
#endif
#if (NUMSTRIPS > 12)
    x1 = *(uint32_t *)(A + 12);
#else
    x1 = 0;
#endif

#if (NUMSTRIPS > 4)
    t = (x ^ (x >> 7)) & AA;
    x = x ^ t ^ (t << 7);
    t = (x ^ (x >> 14)) & CC;
    x = x ^ t ^ (t << 14);
#endif
#if (NUMSTRIPS > 12)
    t = (x1 ^ (x1 >> 7)) & AA;
    x1 = x1 ^ t ^ (t << 7);
    t = (x1 ^ (x1 >> 14)) & CC;
    x1 = x1 ^ t ^ (t << 14);
#endif
    t = (y ^ (y >> 7)) & AA;
    y = y ^ t ^ (t << 7);
    t = (y ^ (y >> 14)) & CC;
    y = y ^ t ^ (t << 14);
#if (NUMSTRIPS > 8)
    t = (y1 ^ (y1 >> 7)) & AA;
    y1 = y1 ^ t ^ (t << 7);
    t = (y1 ^ (y1 >> 14)) & CC;
    y1 = y1 ^ t ^ (t << 14);
#endif
    t = (x & FF) | ((y >> 4) & FF2);
    y = ((x << 4) & FF) | (y & FF2);
    x = t;
    t = (x1 & FF) | ((y1 >> 4) & FF2);
    y1 = ((x1 << 4) & FF) | (y1 & FF2);
    x1 = t;
    *((uint16_t *)(B))      = (uint16_t)(((x & 0xff000000) >> 8 | ((x1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 3))  = (uint16_t)(((x & 0xff0000) >> 16 | ((x1 & 0xff0000) >> 8)));
    *((uint16_t *)(B + 6))  = (uint16_t)(((x & 0xff00) | ((x1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 9))  = (uint16_t)((x & 0xff) | ((x1 & 0xff) << 8));
    *((uint16_t *)(B + 12)) = (uint16_t)(((y & 0xff000000) >> 8 | ((y1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 15)) = (uint16_t)(((y & 0xff0000) | ((y1 & 0xff0000) << 8)) >> 16);
    *((uint16_t *)(B + 18)) = (uint16_t)(((y & 0xff00) | ((y1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 21)) = (uint16_t)((y & 0xff) | ((y1 & 0xff) << 8));
}

//------------------------------------------------------------
// Internal: Callback function called when a transmission completes.
// This is invoked in interrupt context.
bool IRAM_ATTR WWLedS3::flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                      esp_lcd_panel_io_event_data_t* edata,
                                      void* user_ctx) {
    WWLedS3* inst = (WWLedS3*)user_ctx;
    inst->testCount++;
    inst->isDisplaying = false;
    if (inst->isWaiting) {
        inst->isWaiting = false;
        BaseType_t HPTaskAwoken = 0;
        xSemaphoreGiveFromISR(inst->transferSemaphore, &HPTaskAwoken);
        if (HPTaskAwoken == pdTRUE) {
            portYIELD_FROM_ISR(HPTaskAwoken);
        }
    }
    return false;
}

