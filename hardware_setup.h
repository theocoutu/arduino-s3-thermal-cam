// only include this file once
#ifndef HARDWARE_SETUP_H
#define HARDWARE_SETUP_H

void fatal_error(uint8_t err);


// pin connections:
#define PIN_BUTTON_1  (6) //(47)
#define PIN_BUTTON_2  (7) //(46)

#define PIN_I2C_SDA   (5) //(4)
#define PIN_I2C_SCL   (4) //(5)

#define PIN_BACKLIGHT (40)

#define PIN_SPI_SCK   (1)
#define PIN_SPI_MOSI  (2)
#define PIN_SPI_MISO  (39)
#define PIN_TFT_CS    (38)
#define PIN_TFT_DC    (41)
#define PIN_TFT_RST   (42)
#define PIN_TOUCH_CS  (37)
#define PIN_TOUCH_INT (36)


#define TFT_SPI_FREQ          (68000000)    // 68 MHz for ILI9341 writes
#define TFT_SPI_BUS           SPI3_HOST
#define TOUCH_SPI_FREQ        (1000000)     // 1MHz for touch reads
#define TOUCH_SPI_BUS         TFT_SPI_BUS
#define MLX90640_REFRESH_RATE MLX90640_16_HZ // try 4_HZ, 8_HZ, 16_HZ



const uint16_t touch_calib_data[8] = { 3813, 3924, 3832, 209, 260, 3950, 289, 168 };
// (^^ resistive touchscreen calibration parameters ^^)



#include <Adafruit_MLX90640.h>  // thermal image sensor
#include <LovyanGFX.hpp>        // display and touch driver


// Configure display device hardware driver
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ILI9341 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  //lgfx::Light_PWM     _light_instance;
  lgfx::Touch_XPT2046 _touch_instance;
public:
  LGFX(void)
  {
    //
    //  SPI bus
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host    = TFT_SPI_BUS;
      cfg.spi_mode    = 0;
      cfg.freq_write  = TFT_SPI_FREQ;
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = true;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = PIN_SPI_SCK;    //  SCK
      cfg.pin_mosi    = PIN_SPI_MOSI;   //  MOSI
      cfg.pin_miso    = PIN_SPI_MISO;   //  MISO
      cfg.pin_dc      = PIN_TFT_DC;     //  DC

      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    //
    //  Display panel
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs            = PIN_TFT_CS;
      cfg.pin_rst           = PIN_TFT_RST;
      cfg.pin_busy          = -1;
      cfg.panel_width       = 240;
      cfg.panel_height      = 320;
      cfg.offset_x          = 0;
      cfg.offset_y          = 0;
      cfg.offset_rotation   = 0;
      cfg.dummy_read_pixel  = 8;
      cfg.dummy_read_bits   = 1;
      cfg.readable          = true;
      cfg.invert            = false;
      cfg.rgb_order         = false;
      cfg.dlen_16bit        = false;
      cfg.bus_shared        = true;
      _panel_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    //
    //  Backlight (PWM)
    //{
    //  auto cfg        = light_instance_.config();
    //  cfg.pin_bl      = PIN_BACKLIGHT;
    //  cfg.invert      = false;            // HIGH = bright
    //  cfg.freq        = 44100;
    //  cfg.pwm_channel = 7;
    //  light_instance_.config(cfg);
    //  panel_instance_.setLight(&light_instance_);
    //}
    //
    //  Configure touch
    {
      auto cfg = _touch_instance.config();
      cfg.bus_shared  = true;         // Shared SPI bus
      cfg.pin_cs      = PIN_TOUCH_CS; 
      cfg.pin_int     = PIN_TOUCH_INT;
      cfg.pin_rst     = -1;           // -1 = unused
      cfg.x_min       = 0;          // Calibrate these
      cfg.x_max       = 321;
      cfg.y_min       = 0;
      cfg.y_max       = 2421;
      //cfg.offset_rotation = 6;  //  1=swap xy, 2=invert x, 4=inverty
      cfg.freq        = TOUCH_SPI_FREQ;
      cfg.spi_host    = TOUCH_SPI_BUS;
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);  // Link touch to panel
    }
    setPanel(&_panel_instance);
  }
};
//



// Declare device objects
Adafruit_MLX90640 mlx;
LGFX tft; 
LGFX_Sprite main_canvas(&tft);
LGFX_Sprite mlx_canvas(&tft);
//



//
void init_tft_screen(void)
{
  tft.init();
  tft.setRotation(1);     // rotate
  tft.initDMA();          // enable DMA for this display
  tft.setColorDepth(16);  // 16 bit color (RGB565)
  tft.clear(TFT_BLACK);   // clear the display

  // Touchscreen calibration data
  tft.setTouchCalibrate((uint16_t*)touch_calib_data);

  // set up our main_canvas, which will store the image buffer
  main_canvas.setColorDepth(16);
  main_canvas.createSprite(320, 240);
  main_canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  main_canvas.setTextDatum(TL_DATUM);
  main_canvas.setFont(nullptr);

  mlx_canvas.createSprite(32, 24);
}



//
void init_mlx_sensor(void)
{
  Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);

  if ( !mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire) )
  {
    fatal_error(201);
  }
  
  ///*
  Serial.println("Found MLX90640! ");
  //Serial.print("Serial number: ");
  //Serial.print(mlx.serialNumber[0], HEX);
  //Serial.print(mlx.serialNumber[1], HEX);
  //Serial.println(mlx.serialNumber[2], HEX);
  //*/
  
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_REFRESH_RATE);
  Wire.setClock(1000000); // max 1 MHz
}



#endif  // HARDWARE_SETUP_H
