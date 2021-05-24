#include <Arduino.h>
#include "stm32h7xx_hal.h"

#include <stdio.h>

// DMA2D requires the opposite pixel in word order vs my original code
#define FB_REVERSE_PIXEL_ORDER

// Next: 

// dma2d to clear the blocks
// dma2d straight to gpio?

// use an op amp and the adc to do AGC?
// strip out the arduino stuff

// TODO not convinced this is working. Where is the code actually running?
// #define FAST_CODE __attribute__((section(".itcm_text")))
#define FAST_CODE

// #define FAST_DATA __attribute__((section(".dtcm")))
#define FAST_DATA

HardwareTimer *hwTimer = new HardwareTimer(TIM3);

TIM_HandleTypeDef htim3, htim4;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA2D_HandleTypeDef hdma2d;
COMP_HandleTypeDef hcomp1;


bool testBit;

uint32_t nRender = 0, renderMin = 99999, renderMax = 0;
uint64_t totalRenderTimeMicros = 0;

uint32_t interval, duration, fieldRate, previousPulse, previousLine, lineTime;

uint32_t frame = 0, line = 0, frameLength = 0;

bool inVsync = false, lookingForField = true, fieldIsEven = true;

// Characters are 12 pixels wide, 18 pixels high. 30 chars to a line, 16 lines

#define OSD_LINES 288             // not all lines will be visible on ntsc
#define OSD_PIXELS 360
#define PIXELS_PER_LINE 362       // 1 extra pixel at each ond of the output line to always disable the outputs

// Pin 0 connected to white, Pin 1 connected to black

// Black active low (open drain config still turns on the low side when you write 0, it just doesn't use the high side transistor)
// White active high
// #define OSD_BLACK (GPIO_BSRR_BR0 | GPIO_BSRR_BR1)
// #define OSD_WHITE (GPIO_BSRR_BS0 | GPIO_BSRR_BS1)
// #define OSD_TRANSPARENT (GPIO_BSRR_BS0 | GPIO_BSRR_BR1)

// 0: Black active low
// 1: White active low
// 3: Video gate, high = pass, low = block
#define OSD_BLACK (GPIO_BSRR_BR0 | GPIO_BSRR_BS1 | GPIO_BSRR_BR3)
#define OSD_WHITE (GPIO_BSRR_BS0 | GPIO_BSRR_BR1 | GPIO_BSRR_BR3)
#define OSD_TRANSPARENT (GPIO_BSRR_BS0 | GPIO_BSRR_BS1 | GPIO_BSRR_BS3)

// The array has to be 32 byte aligned and also 32 byte padded so that we can manually flush the data cache (!!)
// uint32_t osdLine[PIXELS_PER_LINE + 32/4] __attribute__ ((aligned (32))) = {OSD_TRANSPARENT};
// ALIGN_32BYTES(uint32_t osdLine[PIXELS_PER_LINE+32/4]);


// Test buffer for precomputing a line of text
//uint32_t osdTextLines[18 * PIXELS_PER_LINE + 32/4] __attribute__ ((aligned (32))) = {OSD_TRANSPARENT};

// output buffers for DMA of pixels to OSD gpios
// each line has to be a multiple of 32 bytes so that they are all 32 byte aligned for cache management
#define N_OSD_BUFFER_LINES 2
#define BUFFER_LINE_LENGTH ((((PIXELS_PER_LINE * 4) + 31)/32) * 32 / 4)
ALIGN_32BYTES(uint32_t osdOutputBuffers[N_OSD_BUFFER_LINES][BUFFER_LINE_LENGTH]) = {OSD_TRANSPARENT};


// Line to be sent over DMA to the OSD hardware.
uint8_t lineToOutput = 0;
// uint32_t * dmaOutputBuffer = osdOutputBuffers[lineToOutput];

// flag set by isr handler to signal that the code in loop() should fill the next buffer line
bool renderNeeded = true;

// flag set by comparator ISR to signal that a new field has started
// Used by loop to trigger drawing animated elements
bool newField = false;

// There are 52us per visible line, so 52/360 = 144ns per pixel, equivalent to 6.923 MHz (clock by 34.666 for 240MHz input)
// Use timer4, connect to DMA to transfer odsLine -> gpio pins
// We need a 7us delay (plus a bit?) after we detect hsync before starting to output pixels. 1680 ticks @ 240MHz

// Has to be adjusted by hand to compensate for dma startup delays
#define LINE_START_DELAY 1300
#define OSD_FIRST_VISIBLE_LINE 16

// For easy conversion from max7456, we still match the original character positions
#define CHARACTERS_PER_LINE 30
#define NUMBER_OF_TEXT_LINES 16


// lookup table to translate from 2bpp font encoding to OSD gpio words
const uint32_t pixelCoding[] = {OSD_BLACK, OSD_TRANSPARENT, OSD_WHITE, OSD_TRANSPARENT};

// pixel frame buffer using 4bpp. Lower 2 bits usethe same encoding as the font,
// upper 2 bits are currently ignored (use to implement flashing?)
// stored as words with each word having 8 pixels
#define FB_WORDS ((OSD_PIXELS+7)/8)
uint32_t frameBuffer[OSD_LINES][FB_WORDS];

#define FB_TRANSPARENT 0x11111111


#include "font_default.h"

// This uses an expanded 4bpp format to match the frame buffer. The original font data
// will need to be translated when copying into this array.
FAST_DATA uint8_t fastFont[FONT_NCHAR][18][6];

/** Clear one line of OSD output to transparent
 */
void clearOSDLine(uint32_t *osdLine)
{
  // make sure there isn't something already running
  HAL_DMA2D_PollForTransfer(&hdma2d, 100);

  // config dma2d
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_R2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.Init.BytesSwap = DMA2D_BYTES_REGULAR;
  hdma2d.Init.LineOffsetMode = DMA2D_LOM_PIXELS;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }

  uint32_t pdata = OSD_TRANSPARENT;
  uint32_t dstAddress = (uint32_t)osdLine;

  HAL_StatusTypeDef d2dStatus = HAL_DMA2D_Start(&hdma2d, pdata, dstAddress, PIXELS_PER_LINE, 1); // width in words not pixels

  if (d2dStatus != HAL_OK) {
    Serial.print("failed to start dma2d ");
    Serial.println(d2dStatus);
  }
}

void renderODSLine(uint8_t * fbLine, uint32_t * osdLine)
{
  // config dma2d
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M_PFC;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.Init.BytesSwap = DMA2D_BYTES_REGULAR;
  hdma2d.Init.LineOffsetMode = DMA2D_LOM_BYTES;

  HAL_DMA2D_PollForTransfer(&hdma2d, 1000);
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }

  // setup the input processing
  hdma2d.LayerCfg[DMA2D_FOREGROUND_LAYER].InputColorMode = DMA2D_INPUT_L4;
  hdma2d.LayerCfg[DMA2D_FOREGROUND_LAYER].InputOffset = 0;
  hdma2d.LayerCfg[DMA2D_FOREGROUND_LAYER].AlphaMode = DMA2D_NO_MODIF_ALPHA;

  if (HAL_DMA2D_ConfigLayer(&hdma2d, DMA2D_FOREGROUND_LAYER) != HAL_OK)
  {
    Error_Handler();
  }

  // manually add first and last
  osdLine[0] = OSD_TRANSPARENT;
  osdLine[PIXELS_PER_LINE-1] = OSD_TRANSPARENT;

  #ifndef D_CACHE_DISABLED
  SCB_CleanDCache_by_Addr(&osdLine[0], 4);
  SCB_CleanDCache_by_Addr(&osdLine[PIXELS_PER_LINE-1], 4);
  #endif

  uint32_t pdata = (uint32_t) fbLine;
  uint32_t destAddr = (uint32_t) (osdLine + 1); // skip the first pixel of the line
  if (HAL_DMA2D_Start(&hdma2d, pdata, destAddr, OSD_PIXELS, 1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Clears at least the bounding box, plus any neighbouring pixels in the same frameBuffer words
 * 
 * XXX rewrite with dma2d
 * 
 * Pixels are reset to transparent
 */
void fastClearBlock(const uint16_t xmin, const uint16_t xmax, const uint16_t ymin, const uint16_t ymax)
{
  const uint16_t firstIndex = xmin/8;
  const uint16_t lastIndex = xmax/8;
  const uint16_t nWords = (lastIndex - firstIndex) + 1;

  for(uint16_t line = ymin; line <= ymax; line++)
  {
    uint32_t *fbPtr = &(frameBuffer[line][firstIndex]);
    for(uint16_t words = 0; words < nWords; words++)
    {
      *fbPtr++ = FB_TRANSPARENT;
    }
  }
}

/** Set a pixel in the frame buffer
 * @param value should be one of the font encoding 2bit values (black 0b00, white 0b10, transparent 0bx1)
 * 
 * Each word in the buffer covers 8 pixels, so we need to do a read/modify/write
 * 
 * There are no safety checks in here to keep the speed up, so be careful with input values.
 */
inline void setPixel(const uint16_t x, const uint16_t y, const uint8_t value)
{
  // if (y >= OSD_LINES) return; // yeah, I lied. Debugging

  const uint32_t bufferXIndex = x / 8;
  // if (bufferXIndex >= OSD_WORDS) return;

  #ifdef FB_REVERSE_PIXEL_ORDER
  const uint32_t shiftCount = (x % 8) * 4;
  #else
  const uint32_t shiftCount = (7 - (x % 8)) * 4;
  #endif

  uint32_t tmp = frameBuffer[y][bufferXIndex];

  const uint32_t mask = 0b1111 << shiftCount;

  tmp &= ~mask;
  tmp |= value << shiftCount;

  frameBuffer[y][bufferXIndex] = tmp;
}


/** Draw the specified character into the frame buffer
 * 
 * x,y coordinates in pixels with 0,0 at top left
 * 
 * This version is obviously very innefficient, but it's soon to be replaced with dma2d anyway
 * 
 */
// void drawCharacter(const uint8_t chIndex, const uint16_t x, const uint16_t y)
// {
//   // XXX convert to dma2d

//   // get the character definition from the font table
//   uint32_t * character = font[chIndex];

//   // copy 18 lines of font data into the frame buffer
//   for(uint8_t fontLine=0; fontLine<18; fontLine++)
//   {
//     if (y + fontLine >= OSD_LINES) break; // no point trying to draw off the frame buffer

//     uint32_t row = character[fontLine];

//     // row contains 12 pixels, 2bpp in bits 23 to 0
//     for(int pixelCounter=0; pixelCounter<12; pixelCounter++) {
//       uint8_t pixelValue = (row >> ((11 - pixelCounter) * 2)) & 0b11;
//       setPixel(x+pixelCounter, y+fontLine, pixelValue);
//     }

//   } // for each line
// }

void drawCharacter(const uint8_t chIndex, const uint16_t x, const uint16_t y)
{
  // XXX convert to dma2d

  // get the start of the character data from the font table
  uint8_t * character = fastFont[chIndex][0];

  // copy 18 lines of font data into the frame buffer
  for(uint8_t fontLine=0; fontLine<18; fontLine++)
  {
    if (y + fontLine >= OSD_LINES) break; // no point trying to draw off the frame buffer

    uint16_t lineX = x;

    // each row contains 6 bytes each with 2 pixels
    for(int byteCounter = 0; byteCounter<6; byteCounter++) {
      const uint8_t fontPixelPair = *character++;
      const uint8_t firstP = (fontPixelPair >> 4) & 0b11;
      setPixel(lineX++, y+fontLine, firstP);
      const uint8_t secondP = fontPixelPair & 0b11;
      setPixel(lineX++, y+fontLine, secondP);
    }

  } // for each line
}




/** Draw a string at specified location
 * 
 * Position is in characters with 0,0 in the top left
 * 
 * @param row - the character line the string will be drawn on
 * @param column - the character position to start drawing at
 * 
 */
void drawString(const uint8_t * str, const uint8_t row, const uint8_t column)
{
  uint8_t * cPtr = (uint8_t *)str;
  uint8_t x = column;
  while(*cPtr) {
    // osdFrame[row][x++] = *(cPtr++);  // character based
    drawCharacter(*cPtr++, (x++)*12, row*18);  // pixel based
  }
}

extern "C" {
void DMA1_Stream0_IRQHandler()
{
  HAL_DMA_IRQHandler(&hdma_tim4_ch1); // maybe pull out the needed code to here and save all the indirection?
}
} // extern "C"

void dmaLineCompleteHandler(DMA_HandleTypeDef *hdma)
{
  // Set the buffer to be used for the next output line
  lineToOutput = (lineToOutput + 1) % N_OSD_BUFFER_LINES;
  renderNeeded = true;
}

// interrupt handler gets called when comparator detects a pulse
void ccHandler()
{
  uint32_t pulseStartTicks, pulseStartAdjusted, pulseEndTicks;
  // Serial.print('T');
  // Channel 1 tracks the start of the pulse
  pulseStartTicks = hwTimer->getCaptureCompare(1, TICK_COMPARE_FORMAT);
  // Channel 2 tracks the end of the pulse
  pulseEndTicks = hwTimer->getCaptureCompare(2, TICK_COMPARE_FORMAT);

  // handle wrap arounds
  // Timer3 returns 16 bit values
  if (pulseStartTicks < previousPulse) {
    pulseStartAdjusted = pulseStartTicks + 65536;
  } else {
    pulseStartAdjusted = pulseStartTicks;
  }

  if (pulseEndTicks < pulseStartTicks) pulseEndTicks += 65536;

  interval = pulseStartAdjusted - previousPulse;
  duration = pulseEndTicks - pulseStartTicks;
  previousPulse = pulseStartTicks;

  // expected pulse lengths are:
  //   2us - equalising pulses before and after the vsync 5 or 6 in each group
  //   4us - hsync
  //  27us - vsync, 5 or 6
  // See http://martin.hinner.info/vga/pal.html

  if (duration > 1000 || duration < 10) {
    // not a valid pulse
  } else if (duration > 100) {
    // it's a vsync pulse, but we get multiple per sync, so only act on the first one
    if (!inVsync) {
      inVsync = true;
      newField = true;
      frame++;
      if (line > 0) frameLength = line;
      line = 0;
      fieldRate = 100000000 / lineTime; // a bit approximate, but enough to differentiate between pal and ntsc
      lineTime = 0;
      char buffer[16];
      itoa(frame, buffer, 10);
      drawString((const uint8_t *)buffer, 10, 2);
    }
  } else if (duration > 30) {
    // hsync pulse
    line++;
    lineTime += pulseStartAdjusted - previousLine;
    previousLine = pulseStartTicks;
    lookingForField = true; // since we've seen an hsync, we can start looking for the vsync again

    // hwTimer->pause();

    // Suppress the timer so that we don't start outputting pixels until the visible part of the line
    __HAL_TIM_SET_COUNTER(&htim4, LINE_START_DELAY);

    // Start the DMA channel, configuring the src and dest params

    HAL_DMA_Abort(&hdma_tim4_ch1);

    // HAL_StatusTypeDef status = HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)dmaOutputBuffer, (uint32_t)&(GPIOC->BSRR), PIXELS_PER_LINE);
    HAL_StatusTypeDef status = HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)(osdOutputBuffers[lineToOutput]), (uint32_t)&(GPIOC->BSRR), PIXELS_PER_LINE);
    if (HAL_OK != status) {
      Serial.print("dma start failed ");
      Serial.println(status);
    }


  } else {
    // short "equalising" pulses come before and after the long vsync pulses
    inVsync = false;
    if (lookingForField)
    {
      // this must be the first equalising pulse after seeing hsyncs, so we can use it to check if we're on
      // an odd or even frame by looking at the interval to the last hsync
      fieldIsEven = (interval > 600); // not sure which way around this is
      lookingForField = false;

      // if (fieldIsEven) {
      //   Serial.print("even: ");
      // } else {
      //   Serial.print("odd:  ");
      // }
      // Serial.println(interval);
    }
  }
}


// setup a timer to do pwm input and use
// HAL_StatusTypeDef  HAL_TIMEx_TISelection(TIM_HandleTypeDef *htim, uint32_t TISelection, uint32_t Channel)
// to connect it to the comparator with TIM_TIM3_TI1_COMP1
// static void MX_TIM3_Init(void)
// {
//   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//   TIM_SlaveConfigTypeDef sSlaveConfig = {0};
//   TIM_IC_InitTypeDef sConfigIC = {0};
//   TIM_MasterConfigTypeDef sMasterConfig = {0};

//   htim3.Instance = TIM3;
//   htim3.Init.Prescaler = 0;
//   htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//   htim3.Init.Period = 65535;
//   htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
//   htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//   if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//   if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
//   sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
//   sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
//   sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
//   sSlaveConfig.TriggerFilter = 0;
//   if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
//   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
//   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
//   sConfigIC.ICFilter = 0;
//   if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
//   sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
//   if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   // enable interrupts from the timer
//   sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
//   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//   if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   // connect the output of the comparator to the input of the timer
//   // HAL_TIMEx_TISelection(&htim3, TIM_TIM3_TI1_COMP1, TIM_CHANNEL_1);
// }

/**
 * Setup timer for driving DMA
 */
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  // htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 33; // 240MHz input divided down to give OSD pixel clock ~7MHz
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Plot a simple graph for demo purposes
 * Sinewave with animated amplitude
 */
void plotGraph()
{
  const uint16_t startX = 10, startY = 10;
  const uint16_t width = 120, height = 80;

  const uint8_t AnimationFrames = 25;

  static int8_t direction = 1;

  const uint16_t frameMod = frame % (2 *AnimationFrames);
  if (frameMod == 0) direction *= -1;

  float scale = ((float)((int32_t)(frameMod/2) - AnimationFrames/2) / (AnimationFrames/2)) * direction;
  
  if (scale > 1.0) scale = 1.0;
  if (scale < -1.0) scale = -1.0;

  scale *= (height/2);

  for(uint16_t x = 0; x < width; x++) {
    int16_t y = sin((float)x / width * 3.14 * 4) * scale + height/2;
    setPixel(x + startX, y + startY, 2);
    setPixel(x + startX, startY + height/2, 2);
  }
}

/** Plot a simple graph for demo purposes
 * Sinewave with animated offset
 */
void plotGraphB(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height)
{
  const uint8_t AnimationFrames = 40;
  const float frameOffset = (float)(frame % AnimationFrames) * 6.28 / AnimationFrames;

  for(uint16_t x = 0; x < width; x++) {
    int16_t y = sin((float)x / width * 3.14 * 4 + frameOffset) * (height/2) + height/2;
    setPixel(x + startX, y + startY, 2);
    setPixel(x + startX, startY + height/2, 2);
  }
}

// ============================================================================

void enableClocks()
{
    // enable the comparator
  __HAL_RCC_COMP12_CLK_ENABLE();

  // enable the timer
  // __HAL_RCC_TIM3_CLK_ENABLE();

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  // make sure gpio ports are enabled
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  __HAL_RCC_DMA2D_CLK_ENABLE();
}

void setupGPIO()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Pin for timer4 (tim4 is for the DMA for the OSD pixel output)
  // Will we still need a hardware pin once the dma is linked up internally?
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // setup pins for OSD pixel output
  // All on Port C (one port so we can DMA them together in a single atomic update)
  //   0 = Black, open drain, active low
  //   1 = White, PP, active low (when using bjt)
  //   3 = video gate, PP, high = pass video, low = block video

  // C0 for black, open drain.
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // C1 for bright pixel output (will control a transistor that pulls the video signal high)
  // There are two versions of the output circuit, the mosfet one is active high, the bjt is active low,
  // see the #defs for black, white and transparent mapping.
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // For active low it's critical that the pin is initialised to high for the sync detection to work
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

  // video gate
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**COMP1 GPIO Configuration
  PC5     ------> COMP1_OUT
  PB0     ------> COMP1_INP
  */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_COMP1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // resource config for tim3, used to measure the pulse interval and duration
  /**TIM3 GPIO Configuration
  PA6     ------> TIM3_CH1    // XXX do we still need to use a gpio if the timer is connected to the comp?
  */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void setup()
{
  // uint32_t tStart, tEnd, tFbInit;

    /* Load functions into ITCM RAM */
  extern const unsigned char itcm_text_start;
  extern const unsigned char itcm_text_end;
  extern const unsigned char itcm_data;
  memcpy((void*)&itcm_text_start, &itcm_data, (int) (&itcm_text_end - &itcm_text_start));

  // zero init the osdFrame
  // memset(osdFrame, 0, sizeof(osdFrame));

  // copy the font into fast memory
  // Can't just memcpy anymore, we need to do 2bpp to 4bpp conversion
  for(int i=0; i<FONT_NCHAR; i++)
  {
    for(int line=0; line<18; line++) {
      uint32_t inputRow = font[i][line];
      // input contains 12 pixels at 2bpp in bits 23 to 0
      // output needs 12 pixels at 4bpp in 6 bytes
      for(int byteCount = 0; byteCount<6; byteCount++) {
        uint8_t firstNibble = (inputRow >> ((5 - byteCount) * 4 + 2)) & 0b11;
        uint8_t secondNibble = (inputRow >> ((5 - byteCount) * 4)) & 0b11;
        fastFont[i][line][byteCount] = (firstNibble << 4) | secondNibble;
      }
    }
  }

  delay(1000);

  // Put some content into the frame buffer

  // plotGraph();

  // for(int i=0; i<16; i++) {
  //   if (i< 10) {
  //     osdFrame[i][0] = i + '0';
  //   } else {
  //       osdFrame[i][0] = i - 10 + 'A';
  //   }
  // }

  // for(int i=1; i<30; i++) {
  //   osdFrame[0][i] = (i % 10) + '0';
  // }

  // display the bf logo
  // 4 lines of 24 chars starting at index 160
  // const uint8_t xStart = 3, yStart = 2;
  // uint8_t cIndex = 160;
  // for(int yc = 0; yc < 4; yc++) {
  //   for (int xc = 0; xc < 24; xc++) {
  //     osdFrame[yc+yStart][xc+xStart] = cIndex++;
  //   }
  // }

  enableClocks();

  setupGPIO();

  // set PA1 for output to the LED
  pinMode(D1, OUTPUT);

  pinMode(D24, OUTPUT); // PA4 which is DAC1_OUT1

  analogWrite(D24, 16); // 0.2V threshold for sync pulses


  Serial.begin(460800);
  Serial.println("Hello!");


  // main comp config
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_DAC1_CH1;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_INVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_LOW;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }

  // start the comparator
  if (HAL_COMP_Start(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }


  // config dma2d
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_R2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.Init.BytesSwap = DMA2D_BYTES_REGULAR;
  hdma2d.Init.LineOffsetMode = DMA2D_LOM_BYTES;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }

  // Use dma2d to initialise the frmaebuffer to transparent
  uint32_t pdata = FB_TRANSPARENT;
  uint32_t dstAddress = (uint32_t)&(frameBuffer[0][0]);
  // tStart = micros();
  // took 31us, almost identical to the explicit loop, but can be run in the background with 1us startup time
  HAL_StatusTypeDef d2dStatus = HAL_DMA2D_Start(&hdma2d, pdata, dstAddress, FB_WORDS, OSD_LINES); // width in words not pixels

  if (d2dStatus != HAL_OK) {
    Serial.print("failed to start dma2d ");
    Serial.println(d2dStatus);
  } else {
    // to measure the transfer time we need to loop until the above transaction is complete
    // timeout is in ticks??? How fast is a tick? Looks like ticks are 1ms
    // HAL_DMA2D_PollForTransfer(&hdma2d, 1000);
  }

  // tEnd = micros();
  // tFbInit = tEnd - tStart;
  // char buffer[32];

  // sprintf(buffer, "FB INIT %ld", tFbInit);
  // drawString((const uint8_t *)buffer, 0, 15);

  // Now setup the dma2d for line conversions
  hdma2d.Init.Mode = DMA2D_M2M_PFC;
  HAL_DMA2D_PollForTransfer(&hdma2d, 1000);
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }

  // Need a CLUT from L4 to OSD 32 bpp values.
  ALIGN_32BYTES (uint32_t CLUT_NORMAL[]) = {OSD_BLACK, OSD_TRANSPARENT, OSD_WHITE, OSD_TRANSPARENT,
                                  OSD_BLACK, OSD_TRANSPARENT, OSD_WHITE, OSD_TRANSPARENT,
                                  OSD_BLACK, OSD_TRANSPARENT, OSD_WHITE, OSD_TRANSPARENT,
                                  OSD_BLACK, OSD_TRANSPARENT, OSD_WHITE, OSD_TRANSPARENT};

  #ifndef D_CACHE_DISABLED
  SCB_CleanDCache_by_Addr(CLUT_NORMAL, sizeof(CLUT_NORMAL));
  #endif

  DMA2D_CLUTCfgTypeDef clutCfg;

  clutCfg.CLUTColorMode = DMA2D_CCM_ARGB8888;
  clutCfg.pCLUT = (uint32_t *)CLUT_NORMAL;
  clutCfg.Size = 16;

  if (HAL_DMA2D_CLUTStartLoad(&hdma2d, &clutCfg, DMA2D_FOREGROUND_LAYER) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_DMA2D_PollForTransfer(&hdma2d, 1000);

  drawString((const uint8_t*)"DMA2D IS GO!", 6, 10);

  // setup timer4
  MX_TIM4_Init();

  // setup the dma that tim4 will drive
  hdma_tim4_ch1.Instance = DMA1_Stream0;
  hdma_tim4_ch1.Init.Request = DMA_REQUEST_TIM4_CH1;
  hdma_tim4_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim4_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim4_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim4_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim4_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim4_ch1.Init.Mode = DMA_NORMAL; // Stop after outputting the line of pixels
  hdma_tim4_ch1.Init.Priority = DMA_PRIORITY_MEDIUM;
  hdma_tim4_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_tim4_ch1.XferCpltCallback = dmaLineCompleteHandler;  // NEW needs testing
  if (HAL_DMA_Init(&hdma_tim4_ch1) != HAL_OK)
  {
    Error_Handler();
  }


  // setup the measurement timer (timer3) with arduino calls for now
  #define CHANNEL1 1
  hwTimer->setMode(CHANNEL1, TIMER_INPUT_FREQ_DUTY_MEASUREMENT);
  hwTimer->setPrescaleFactor(24);

  hwTimer->attachInterrupt(2, ccHandler);

  // config timer 3
  // MX_TIM3_Init();

  // start the timer
  // HAL_TIM_Base_Start(&htim3); // XXX is this needed?
  // HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1); // XXX need to start channel 2 as well?

  // Connect the comp to the timer
  // Need a TIM handle to pass to the TISelection call, not sure if it actually uses anything other
  // than the Instance.
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIMEx_TISelection(&htim3, TIM_TIM3_TI1_COMP1, TIM_CHANNEL_1);
  HAL_TIMEx_TISelection(&htim3, TIM_TIM3_TI1_COMP1, TIM_CHANNEL_2);

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  // HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  // Turn things on

  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  __HAL_LINKDMA(&htim4,hdma[TIM_DMA_ID_CC1],hdma_tim4_ch1);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1); // required


  // start the timer
  hwTimer->resume();

}

// is fast code working? It seems to make things slower ?!
// FAST_CODE void renderLine()
// {
//   uint32_t * buffer = osdOutputBuffers[(lineToOutput + 1) % N_OSD_BUFFER_LINES];
//   uint16_t lineToRender = line + 2; // may need to store lineToRender in the dma interrupt handler to avoid race conditions
//   if (lineToRender < OSD_FIRST_VISIBLE_LINE || lineToRender > (OSD_FIRST_VISIBLE_LINE + 288))
//   {
//     // need to blank the output buffer
//     for(int i=0; i<PIXELS_PER_LINE; i++)
//     {
//       buffer[i] = OSD_TRANSPARENT;
//     }
//   } else {
//     uint32_t osdY = lineToRender - OSD_FIRST_VISIBLE_LINE;
//     uint32_t *osdLine = &(frameBuffer[osdY][0]);

//     uint32_t outputIndex = 1;

//     // for each word in the line
//     for (uint16_t i=0; i<FB_WORDS; i++)
//     {
//       uint32_t pixelGroup = osdLine[i];
//       // expand the 8 pixels
//       for(int j=0; j<8; j++) 
//       {
//         uint8_t pixel = (pixelGroup >> ((7-j) * 4)) & 0b11; // ignore the higher bits
//         buffer[outputIndex++] = pixelCoding[pixel];
//       }
//     }
//     // add the special beginning and end of line values
//     buffer[0] = OSD_TRANSPARENT;
//     buffer[PIXELS_PER_LINE-1] = OSD_TRANSPARENT;
//   }

//   // force the buffer line out of the data cache
//   #ifndef D_CACHE_DISABLED
//   SCB_CleanDCache_by_Addr((uint32_t*)buffer, PIXELS_PER_LINE*4);
//   #endif

// }

FAST_CODE void renderLine()
{
  uint32_t * buffer = osdOutputBuffers[(lineToOutput + 1) % N_OSD_BUFFER_LINES];
  uint16_t lineToRender = line + 2; // may need to store lineToRender in the dma interrupt handler to avoid race conditions
  if (lineToRender < OSD_FIRST_VISIBLE_LINE || lineToRender > (OSD_FIRST_VISIBLE_LINE + 288))
  {
    // need to blank the output buffer
    clearOSDLine(buffer);

    // HAL_DMA2D_PollForTransfer(&hdma2d, 100);
    // for(int i=0; i<PIXELS_PER_LINE; i++) {
    //   if (buffer[i] != OSD_TRANSPARENT) {
    //     Serial.println("clear failed");
    //     break;
    //   }
    // }

  } else {
    uint32_t osdY = lineToRender - OSD_FIRST_VISIBLE_LINE;
    uint32_t *osdLine = &(frameBuffer[osdY][0]);

    #ifndef D_CACHE_DISABLED
    SCB_CleanDCache_by_Addr(osdLine, FB_WORDS*4);
    #endif

    renderODSLine((uint8_t *)osdLine, buffer);

  }

  // force the buffer line out of the data cache
  // #ifndef D_CACHE_DISABLED
  // SCB_CleanDCache_by_Addr(buffer, PIXELS_PER_LINE*4);
  // #endif

}


#define LED_INTERVAL_MS 500

void loop() {

  static uint32_t ledLastUpdate = 0;
  static uint8_t ledState = 0;

  // Render the next line if needed
  if (renderNeeded) {
    uint32_t tStart = micros();
    renderLine();
    renderNeeded = false;
    uint64_t tEnd = micros();
    if (tEnd < tStart) tEnd += (uint64_t)1 << 32; // compensate for wrap arounds
    uint32_t tElapsed = tEnd - tStart;
    totalRenderTimeMicros += tElapsed;
    nRender++;
    if (tElapsed > renderMax) renderMax = tElapsed;
    if (tElapsed < renderMin) renderMin = tElapsed;
  }

  // redraw animations if we're on a new field
  if (newField) {
    newField = false;
  
    const uint16_t startX = 10, startY = 10;
    const uint16_t width = 120, height = 80;

    fastClearBlock(startX, startX+width, startY, startY+height);
    plotGraph();

    {
      const uint16_t startX = 250, startY = 175;
      const uint16_t width = 80, height = 45;

      fastClearBlock(startX, startX+width, startY, startY+height);
      plotGraphB(startX, startY, width, height);
    }
  }


  uint32_t now = millis();


  if ((now > (ledLastUpdate + LED_INTERVAL_MS)))
  {
    ledState = !ledState;
    digitalWrite(D1, ledState);
    ledLastUpdate = now;
    // Serial.println("L");
    char buff[12];
    drawString((const uint8_t*)"MIN", 8, 2);
    itoa(renderMin, buff, 10);
    drawString((const uint8_t*)buff, 8, 6);

    drawString((const uint8_t*)"MAX", 8, 10);
    itoa(renderMax, buff, 10);
    drawString((const uint8_t*)buff, 8, 14);

    uint32_t average = totalRenderTimeMicros * 100 / nRender;
    uint32_t aveDigits = average/100;
    uint32_t aveFraction = average - (aveDigits * 100);
    drawString((const uint8_t*)"AVG", 8, 18);
    itoa(aveDigits, buff, 10);
    drawString((const uint8_t*)buff, 8, 22);
    drawString((const uint8_t*)".", 8, strlen(buff)+22);
    uint8_t col = strlen(buff)+23;
    uint8_t firstChar = aveFraction/10;
    uint8_t secondChar = aveFraction - (firstChar * 10);
    buff[0] = firstChar + '0';
    buff[1] = secondChar + '0';
    buff[2] = 0;
    drawString((const uint8_t*)buff, 8, col);


    totalRenderTimeMicros = 0;
    nRender = 0;
  }



  // needs a last debug time variable
  // Serial.print(interval);
  // Serial.print(' ');
  // Serial.print(duration);
  // Serial.print(' ');
  // Serial.print(frame);
  // Serial.print(' ');
  // Serial.print(frameLength);
  // Serial.print(' ');
  // Serial.println(fieldRate);
}

