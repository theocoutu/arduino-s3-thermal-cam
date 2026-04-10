/**
 *  Théodore Coutu
 *  2026-04-09
 *
 *  This project implements a thermal imaging camera using ESP32-S3 and
 *  MLX90640 thermal imaging sensor.
 *
 *  ILI9341 TFT display (320x240) over SPI with DMA, and
 *  XPT2046 resistive touch controller on shared SPI bus.
 *  MLX90640 thermal imaging sensor (32x24) over I2C. 
 *  
 *  Each pixel from the MLX90640 is scaled up by 9 to fit on the display:
 *  288 x 216 with 24 pixels on the right for a color scale bar, and a
 *  32 pixel info bar at the bottom, showing min, max, and avg temperatures
 *  for the current frame, and the temperature for the selected pixel, which
 *  can be changed by touching the screen. The colours scale automatically, 
 *  across the entire 16-bit RGB colour scale using a custom function.
 *  
 */


// control minimum time per frame: @16hz, 62.5ms/frame, @8Hz 125ms/frame
#define MS_PER_FRAME  (65)

// milliseconds to wait when debouncing a button press, 50ms is good
#define DEBOUNCE_DELAY_MS  (50)

// Smoothing factor for exponential moving average of min/max temps, 
// using Brown's Simple Exponential Smoothing formula:
// (from https://people.duke.edu/~rnau/411avg.htm)
//        Y(t+1) =   Y(t)   + alpha * change
// lower=smoother, higher=faster response
#define TEMP_SMOOTH_ALPHA     0.2f          // 0.05f – 0.5f works well



// Include libraries
#include "hardware_setup.h"      // My hardware definitions
#include "freertos/FreeRTOS.h"   // FreeRTOS
#include "freertos/task.h"       // tasks
#include "freertos/semphr.h"     // semaphore (passing data between cores)



// consts
const TickType_t frame_period = pdMS_TO_TICKS(MS_PER_FRAME);
const TickType_t btn_debounce_period = pdMS_TO_TICKS(DEBOUNCE_DELAY_MS);



// global variables
float min_temp       = -40.0;          // °C, blue
float max_temp       = 300.0;          // °C, red
float min_temp_prev  = -40.0;          // °C
float max_temp_prev  = 300.0;          // °C

float avg_temp = 0.0;

uint32_t last_frame_millis = 0; // to calculate time per frame
uint16_t touched_cell = 400;  // to get the temperature at a point 
float touched_temp = 0.0;
//uint16_t touched_col  = touched_cell % 32;  // to get X coord of touched point
//uint16_t touched_row  = touched_cell / 32;  // to get Y coord of touched point

float mlx_frame_sensor[32 * 24];    // buffer to read frame of temperature floats

float mlx_frame_print[32 * 24];     // buf to output over serial
float mlx_frame_display[32 * 24];   // buf to display onscreen

SemaphoreHandle_t print_mutex;      // Protect buffer when swapping
SemaphoreHandle_t display_mutex;    // ^^


volatile bool enable_print =  true;     // Whether to output data over Serial
volatile bool enable_sensor = true;     // Whether to fetch new data and display it



// FreeRTOS tasks
TaskHandle_t btn_1_task_handle = nullptr;
TaskHandle_t btn_2_task_handle = nullptr;
TaskHandle_t print_task_handle = nullptr;
TaskHandle_t mlx_task_handle = nullptr;
TaskHandle_t display_task_handle = nullptr;
void btn_1_task(void *pvParameters);
void btn_2_task(void *pvParameters);
void print_task(void *pvParameters);
void mlx_task(void *pvParameters);
void display_task(void *pvParameters);



// function declarations
void fatal_error(uint8_t err);

void init_tft_screen(void);
void init_mlx_sensor(void);

bool get_mlx_frame(float *dest);
void draw_mlx_frame_on_screen(float *frame_data);
void write_text_to_display(void);
void draw_touched_cell_on_screen(uint16_t cell, float temp);
void process_touch_input(uint16_t *cell);
uint16_t map_temp_to_color(float temp, float temp_min, float temp_max);
void print_frame(float *frame_print);



// Arduino code
void setup(void)
{
  Serial.begin(115200);

  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);

  print_mutex = xSemaphoreCreateMutex();
  display_mutex = xSemaphoreCreateMutex();

  init_tft_screen();
  init_mlx_sensor();

  // Task that reads button 1 (Core 0)
  xTaskCreatePinnedToCore(
    btn_1_task,
    "Btn_1_Task",
    4096,
    nullptr,
    2,           // priority
    &btn_1_task_handle,
    0            // core 0
  );

  // Task that reads button 2 (Core 0)
  xTaskCreatePinnedToCore(
    btn_2_task,
    "Btn_2_Task",
    4096,
    nullptr,
    2,           // priority
    &btn_2_task_handle,
    0            // core 0
  );

  // Task that prints to serial (Core 0)
  xTaskCreatePinnedToCore(
    print_task,
    "Print_Task",
    8192,
    nullptr,
    1,           // priority
    &print_task_handle,
    0            // core 0
  );

  // Task that gets new sensor data (Core 1)
  xTaskCreatePinnedToCore(
    mlx_task,
    "Mlx_Task",
    8192,
    nullptr,
    3,           // priority
    &mlx_task_handle,
    1            // core 1
  );

  // Task that prints to serial (Core 1)
  xTaskCreatePinnedToCore(
    display_task,
    "Display_Task",
    8192,
    nullptr,
    2,           // priority
    &display_task_handle,
    1            // core 1
  );
  //tskNO_AFFINITY // or can be run on both CPUs

}



void loop(void)
{
  // everything is handled in tasks
  vTaskSuspend(NULL); // suspend the current task
  //vTaskDelay( (uint32_t)-1 / portTICK_PERIOD_MS); // or maximum delay
}



// Unrecoverable error, reset the system.
// 201: failure initializing the sensor.
// 202: failure getting frame data from sensor.
void fatal_error(uint8_t err)
{
  char *msg;
  switch (err)
  {
    case 201:
      msg = "mlx.begin(): Can't connect! Check connection. Resetting.";
      break;
    case 202:
      msg = "mlx.getFrame(): Can't get frame! Check connection. Resetting.";
      break;
    default:
      msg = "MLX Thermal Camera: Unknown fatal error. Resetting.";
      break;
  }
  Serial.println(msg);
  delay(500);
  ESP.restart();
}



// get a new frame and put it in *dest
bool get_mlx_frame(float *dest)
{
  // Get thermal sensor data
  if (mlx.getFrame(dest) != 0)
  {
    fatal_error(202);
    return false;
  }
  return true;
}



// get touch input, if it falls within the image then update *cell
void process_touch_input(uint16_t* cell)
{
  int32_t x, y;
  const uint8_t pixel_scale = 9;
  
  if ( !tft.getTouch(&x, &y) )
  {
    return;
  }

  if ( (y <= 216) && (x <= 288) )
  {
    uint16_t touched_col = x/pixel_scale;
    uint16_t touched_row = y/pixel_scale;
    *cell = (uint16_t)((touched_row*32) + touched_col);
    //cell = (touched_row*32) + 31-touched_col; // to reverse X
  }
  
}



//
void draw_mlx_frame_on_screen(float *frame_data)
{
  // get per frame min, max, and avg
  float frame_min = frame_data[0];
  float frame_max = frame_data[0];
  float frame_avg = 0.0;

  for (uint16_t ti = 0; ti < 768; ti += 1)
  {
    float t = frame_data[ti];
    frame_avg += t;
    if (t < frame_min) frame_min = t;
    if (t > frame_max) frame_max = t;
  }

  // Draw scaled MLX data
  for (uint8_t h = 0; h < 24; h++)
  {
    for (uint8_t w = 0; w < 32; w++)
    {
      float tf = frame_data[(h * 32) + (31-w)]; 
      uint16_t color = map_temp_to_color(tf, frame_min, frame_max);
      mlx_canvas.drawPixel(w, h, color);
    }
  }
  main_canvas.fillSprite(TFT_BLACK);

  // scale up the small image, centred at 144, 108 (plus 4 for offset)
  //  which is the centre of (320*0.9),(240*0.9)   (plus 4)
  mlx_canvas.pushRotateZoom(&main_canvas, 149, 112, 0, 9.0, 9.0);

  // calculate average
  frame_avg /= 768.0;
  avg_temp = frame_avg;

  // Smooth global min/max toward the frame min/max
  // using TEMP_SMOOTH_ALPHA
  min_temp = min_temp + (TEMP_SMOOTH_ALPHA * (frame_min - min_temp) );
  max_temp = max_temp + (TEMP_SMOOTH_ALPHA * (frame_max - max_temp) );
  // Or simply update the temperature scale immediately:
  //min_temp = frame_min;
  //max_temp = frame_max;
}
//



//
void write_text_to_display()
{
  // Right-side color bar: use the free margin on the right
  for (uint8_t y = 2; y < 214; y++)
  {
    // y = 2 -> max_temp color, y = 214 -> min_temp color
    float frac = 1.0f - (float)(y-2) / 212.0f;
    float t    = min_temp + frac * (max_temp - min_temp);
    
    uint16_t color = map_temp_to_color(t, min_temp, max_temp);

    main_canvas.drawFastHLine(311, y, 7, color);
  }

  // Draw border around color bar (x,y,w,h,color)
  main_canvas.drawRect(310, 1, 9, 215, TFT_WHITE);
  

  // deg C in the middle of the color scale
  main_canvas.setTextDatum(TR_DATUM);
  main_canvas.drawString("C", 306, 108);
  main_canvas.drawCircle(296, 106, 2, TFT_WHITE);


  // print min, max, avg temps
  char min_max_avg_label[64];
  snprintf(
    min_max_avg_label, 64, 
    "Min: %3.1f | Max: %3.1f | Avg: %3.1f ",
    min_temp, max_temp, avg_temp
  );
  main_canvas.setTextDatum(TL_DATUM);
  main_canvas.drawString(min_max_avg_label, 1, 218);


  draw_touched_cell_on_screen(touched_cell, touched_temp);

  // print temperature of touched cell
  char touched_cell_label[64];
  snprintf(
    touched_cell_label, 64, "Touched cell at (%d,%d): %3.1f ",
    touched_cell % 32, touched_cell / 32, touched_temp
  );
  main_canvas.setTextDatum(BL_DATUM);
  main_canvas.drawString(touched_cell_label, 1, 238);


  char sensor_enabled_label[32];
  snprintf(
    sensor_enabled_label, 32, "Sensor: %s",
    enable_sensor?"ON":"Off"
  );
  main_canvas.setTextDatum(TR_DATUM);
  main_canvas.drawString(sensor_enabled_label, 318, 218);

  char output_enabled_label[32];
  snprintf(
    output_enabled_label, 32, "Output: %s",
    enable_print?"ON":"Off"
  );
  main_canvas.setTextDatum(BR_DATUM);
  main_canvas.drawString(output_enabled_label, 318, 238);
}



// draw a box around the touched cell
void draw_touched_cell_on_screen(uint16_t cell, float temp)
{
  uint16_t touched_col = cell % 32;
  uint16_t touched_row = cell / 32;
  const uint8_t pixel_scale = 9;

  //get the touched cell
  main_canvas.drawRect(
    touched_col * pixel_scale, 
    touched_row * pixel_scale, 
    pixel_scale + 1, 
    pixel_scale + 1, 
    TFT_WHITE
  );
}
//



// custom color mapping function, inspired by
// https://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients
uint16_t map_temp_to_color(float temp, float temp_min, float temp_max)
{
  // Normalize temperature to 0.0 - 1.0 range
  float value = (temp - temp_min) / (temp_max - temp_min);
  
  // Clamp value between 0 and 1
  if (value < 0.0f) value = 0.0f;
  if (value > 1.0f) value = 1.0f;

  uint16_t r, g, b; 

  // Define seven color points:
  // 0.000: Black  ( 0 ,  0 ,  0 )
  // 0.166: Blue   ( 0 ,  0 , 255)
  // 0.333: Cyan   ( 0 , 255, 255)
  // 0.500: Green  ( 0 , 255,  0 )
  // 0.666: Yellow (255, 255,  0 )
  // 0.833: Red    (255,  0 ,  0 )
  // 1.000: White  (255, 255, 255)
  // and scale for 0-255 in between:
  
  // Black to Blue
  if (value < 0.166f)
  {
    float t = value / 0.166f;
    r = 0;
    g = 0;
    b = (uint8_t)(t * 255);
  }

  // Blue to Cyan
  else if (value < 0.333f)
  {
    float t = (value - 0.166f) / 0.167f;
    r = 0;
    g = (uint8_t)(t * 255);
    b = 255;
  }
  
  // Cyan to Green
  else if (value < 0.500f)
  {
    float t = (value - 0.333f) / 0.167f;
    r = 0;
    g = 255;
    b = (uint8_t)((1.0f - t) * 255);
  }

  // Green to Yellow
  else if (value < 0.666f)
  {
    float t = (value - 0.500f) / 0.166f;
    r = (uint8_t)(t * 255);
    g = 255;
    b = 0;
  }

  // Yellow to Red
  else if (value < 0.833f)
  {
    float t = (value - 0.666f) / 0.167f;
    r = 255;
    g = (uint8_t)((1.0f - t) * 255);
    b = 0;
  }

  // Red to White
  else // value >= 0.833f
  {
    float t = (value - 0.833f) / 0.167f;
    r = 255;
    g = (uint8_t)(t * 255);
    b = (uint8_t)(t * 255);
  }
  
  // to rgb565:
  //r = (r >> 3) & 0x1F;
  //g = (g >> 2) & 0x3F;
  //b = (b >> 3) & 0x1F;
  r = r * 31/255; 
  g = g * 63/255;
  b = b * 31/255;

  uint16_t clr_num = (r << 11) | (g << 5) | (b);
  return clr_num;
}
//



//
void print_frame(float *frame_print)
{
  Serial.printf(
    "%c, %3.1f, %3.1f, %3.1f, %d, %3.1f, ", 
    255, min_temp, max_temp, avg_temp, touched_cell, touched_temp
  );
  uint16_t count = 0;

  // get sensor data
  for (uint8_t h = 0; h < 24; h++)
  {
    for (uint8_t w = 0; w < 32; w++)
    {
      Serial.printf(
        "%3.1f, ",
        frame_print[(h * 32) + (31-w)]
      );
      
      count += 1;
    }
  }

  Serial.printf("%d, 768\n", count);
}
//





// typical Arduino debounce using millis() adapted to FreeRTOS delays
// inverts enable_print on button 1 press
void btn_1_task(void *pvParameters)
{
  TickType_t last_wake_btn_1 = xTaskGetTickCount();
  bool last_btn_1_state = HIGH; // active-low
  bool toggled_1;

  while (1)
  {
    toggled_1 = false;

    // read and debounce button 1
    bool current_btn_1_state = digitalRead(PIN_BUTTON_1);
    if ( last_btn_1_state == HIGH  &&  current_btn_1_state == LOW )
    {
      vTaskDelay( btn_debounce_period );

      if ( digitalRead(PIN_BUTTON_1) == LOW )
      {
        toggled_1 = true;

        while ( digitalRead(PIN_BUTTON_1) == LOW )
        {
          vTaskDelay( btn_debounce_period );
        }
      }
    }

    last_btn_1_state = current_btn_1_state;

    if (toggled_1)
    {
      // flip boolean
      enable_print ^= 1;
    }

    //
    vTaskDelayUntil(&last_wake_btn_1, btn_debounce_period);
  }
}



// Copy of btn_1_task
// inverts enable_sensor on button 2 press
void btn_2_task(void *pvParameters)
{
  TickType_t last_wake_btn_2 = xTaskGetTickCount();
  bool last_btn_2_state = HIGH; // active-low

  while (1)
  {
    bool toggled_2 = false;

    // read and debounce button 1
    bool current_btn_2_state = digitalRead(PIN_BUTTON_2);
    if ( last_btn_2_state == HIGH  &&  current_btn_2_state == LOW )
    {
      vTaskDelay( btn_debounce_period );

      if ( digitalRead(PIN_BUTTON_2) == LOW )
      {
        toggled_2 = true;

        while ( digitalRead(PIN_BUTTON_2) == LOW )
        {
          vTaskDelay( btn_debounce_period );
        }
      }
    }

    last_btn_2_state = current_btn_2_state;

    if (toggled_2)
    {
      // flip boolean
      enable_sensor ^= 1;
    }

    //
    vTaskDelayUntil(&last_wake_btn_2, btn_debounce_period);
  }
}



// outputs every new frame over serial if enable_print is true
void print_task(void *pvParameters)
{
  TickType_t last_wake_print = xTaskGetTickCount();
  bool do_print_frame = true;

  while (1)
  {
    if (enable_print)
    {
      // continuous output when the sensor is running
      if (enable_sensor)
      {
        do_print_frame = true;
      }
      
      // output a frame
      if (do_print_frame)
      {
        float frame[32*24];

        if (xSemaphoreTake(print_mutex, portMAX_DELAY) == pdTRUE)
        {
          memcpy(frame, mlx_frame_print, sizeof(mlx_frame_print));
          xSemaphoreGive(print_mutex);
        }

        print_frame(frame);

        // if the sensor is off, don't print another frame
        if (!enable_sensor)
        {
          do_print_frame = false;
        }

      }
    }

    // if enable_print is false, enable the sensor for next time
    else
    {
      do_print_frame = true;
    }

    vTaskDelayUntil(&last_wake_print, frame_period);
  }
}



// read sensor data into frame buffers
void mlx_task(void *pvParameters)
{
  TickType_t last_wake_mlx = xTaskGetTickCount();

  while (1)
  {
    if (enable_sensor)
    {
      if (get_mlx_frame(mlx_frame_sensor))
      {
        if (enable_print)
        {
          // Copy to print buffer
          if (xSemaphoreTake(print_mutex, portMAX_DELAY) == pdTRUE)
          {
            memcpy(
              mlx_frame_print, mlx_frame_sensor, sizeof(mlx_frame_print)
            );
            xSemaphoreGive(print_mutex);
          }
        }

        // Copy to display buffer
        if (xSemaphoreTake(display_mutex, portMAX_DELAY) == pdTRUE)
        {
          memcpy(
            mlx_frame_display, mlx_frame_sensor, sizeof(mlx_frame_display)
          );
          xSemaphoreGive(display_mutex);
        }
      }
    }

    vTaskDelayUntil(&last_wake_mlx, frame_period);
  }
}



// handle reading touch input from and writing data to the display device
void display_task(void *pvParameters)
{
  TickType_t last_wake_display = xTaskGetTickCount();
  float frame[32 * 24];   // local copy

  while (1)
  {
    // Grab a snapshot of the display buffer
    if (enable_sensor)
    {
      if (xSemaphoreTake(display_mutex, portMAX_DELAY) == pdTRUE)
      {
        memcpy(frame, mlx_frame_display, sizeof(frame));
        xSemaphoreGive(display_mutex);
      }
      // if 'enable_sensor' is false, then 'frame' contains the last image
    }


    // start by drawing the sensor data
    draw_mlx_frame_on_screen(frame);

    // process_touch_input AFTER draw_mlx_frame
    process_touch_input(&touched_cell);
    touched_temp = frame[touched_cell];
    
    // finally, add the onscreen text below the image, and the color scale
    write_text_to_display();


    // send the finished image to the display
    tft.startWrite();
    main_canvas.pushSprite(0, 0);
    tft.endWrite();

    // display runs at 2x sensor rate
    vTaskDelayUntil(&last_wake_display, frame_period/2);
  }
}

