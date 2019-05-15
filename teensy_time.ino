/*  Glenn's Time visualizer.
 *   Not going to use the audio library...just going to do analog reads.   
 */

#include <SmartLEDShieldV4.h>  // comment out this line for if you're not using SmartLED Shield V4 hardware (this line needs to be before #include <SmartMatrix3.h>)
#include <SmartMatrix3.h>
#include <FastLED.h>

#define COLOR_DEPTH 24                  // known working: 24, 48 - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24
const uint8_t kMatrixWidth = 64;        // known working: 32, 64, 96, 128
const uint8_t kMatrixHeight = 32;       // known working: 16, 32, 48, 64
const uint8_t kRefreshDepth = 36;       // known working: 24, 36, 48
const uint8_t kDmaBufferRows = 4;       // known working: 2-4, use 2 to save memory, more to keep from dropping frames and automatically lowering refresh rate
const uint8_t kPanelType = SMARTMATRIX_HUB75_32ROW_MOD16SCAN; // use SMARTMATRIX_HUB75_16ROW_MOD8SCAN for common 16x32 panels, or use SMARTMATRIX_HUB75_64ROW_MOD32SCAN for common 64x64 panels
const uint8_t kMatrixOptions = (SMARTMATRIX_OPTIONS_NONE);      // see http://docs.pixelmatix.com/SmartMatrix for options
const uint8_t kBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);
const uint8_t kScrollingLayerOptions = (SM_SCROLLING_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);
//SMARTMATRIX_ALLOCATE_SCROLLING_LAYER(scrollingLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kScrollingLayerOptions);

#define ADC_INPUT_PIN   A2

//#define DEBUG_ON

#define NUM_SAMPLES 64
int time_samples[NUM_SAMPLES];

int sample_bias = 1024/3;  // my circuit bias up with a 1/3 voltage divider.
int pixels_per_unit = sample_bias/32;  // 32 pixels of resolution...

// pass in raw 0-1023 from ADC
// returns 0-31 for matrix
int map_sample( int input )
{
  int mapped_sample;

  // start by taking out DC bias.  This will make negative #s...
  mapped_sample = input - sample_bias;

  // add in gain.
  mapped_sample = mapped_sample / pixels_per_unit;
  
  // center on 16.
  mapped_sample = mapped_sample + 16;

  // and clip.
  if (mapped_sample > 31) mapped_sample = 31;
  if (mapped_sample < 0) mapped_sample = 0;

  return mapped_sample;
}

void show_samples_lines( void )
{
  int x;
  int y;
  int last_x=0;
  int last_y=16;
  rgb24 line_color = {0,0,200};
  rgb24 black = {0,0,0};

  backgroundLayer.fillScreen(black);

  last_y=map_sample(time_samples[x]);
  last_x = 0;
  
  for (x=1; x < NUM_SAMPLES; x++)
  {
    y=map_sample(time_samples[x]);
    backgroundLayer.drawLine(last_x,last_y,x,y,line_color);
    last_x = x;
    last_y = y;
  }

  backgroundLayer.swapBuffers();
}

void collect_samples( void )
{
  int i;

  #ifdef DEBUG_ON
  unsigned long start_time;
  unsigned long end_time;
  float ms_per_sample;
  start_time = millis();
  Serial.print("Start time: ");
  Serial.println(start_time);
  #endif
  
  for (i=0; i<NUM_SAMPLES; i++)
  {
    time_samples[i] = analogRead(ADC_INPUT_PIN);
    delay(1);  
  }

  #ifdef DEBUG_ON
  end_time = millis();
  Serial.print("End time: ");
  Serial.println(end_time);
  Serial.print("Total time to collect samples: ");
  Serial.print(end_time - start_time);
  Serial.print(" ms.  ");
  ms_per_sample = (end_time - start_time) / NUM_SAMPLES;
  Serial.print(ms_per_sample);
  Serial.print(" per sample, or ");
  Serial.print(1000/ms_per_sample);
  Serial.println(" Hz");
  #endif
  
}

void print_samples( void )
{
  int i;

  for (i = 0; i < NUM_SAMPLES; i++)
  {
    Serial.println(time_samples[i]);
  }
  Serial.println("===============");
  
}

void setup()
{
    Serial.begin(9600);

    // Initialize Matrix
    matrix.addLayer(&backgroundLayer); 
    //matrix.addLayer(&scrollingLayer);
    matrix.begin();

    matrix.setBrightness(255);
}


void loop()
{
    collect_samples();
    show_samples_lines();

    //delay(10);
    
    #if 0
    Serial.println("Hit a key to collect samples");
    while (!Serial.available());
    collect_samples();
    print_samples();
    while (Serial.available())
    {
      char c = Serial.read();
    }
    #endif
    
}
