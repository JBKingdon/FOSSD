#include <Arduino.h>
#include "stm32h7xx_hal.h"

#include <stdio.h>

// Next: 

// use an op amp and the adc to do AGC?
// strip out the arduino stuff

// TODO not convinced this is working. Where is the code actually running?
// #define FAST_CODE __attribute__((section(".itcm_text")))
#define FAST_CODE

#define FAST_DATA __attribute__((section(".dtcm")))
// #define FAST_DATA

TIM_HandleTypeDef htim3, htim4;
HardwareTimer *hwTimer = new HardwareTimer(TIM3);
DMA_HandleTypeDef hdma_tim4_ch1;

bool testBit;

uint32_t nRender = 0, renderMin = 99999, renderMax = 0;
uint64_t totalRenderTimeMicros = 0;

uint32_t interval, duration, fieldRate, previousPulse, previousLine, lineTime;

uint32_t frame = 0, line = 0, frameLength = 0;

bool inVsync = false, lookingForField = true, fieldIsEven = true;

// pixel buffer for testing. Characters are 12 pixels wide, 30 chars to a line.
// We're going to dma the buffer to the gpio bsrr register, so the bits per pixel has to match that, even though
// we're only using 1 or 2 of the bits.
#define PIXELS_PER_LINE 362       // 1 extra pixel at each ond of the line to always disable the outputs

// Pin 0 connected to white, Pin 1 connected to black

// Black active low (open drain config still turns on the low side when you write 0, it just doesn't use the high side transistor)
// White active high
// #define OSD_BLACK (GPIO_BSRR_BR0 | GPIO_BSRR_BR1)
// #define OSD_WHITE (GPIO_BSRR_BS0 | GPIO_BSRR_BS1)
// #define OSD_TRANSPARENT (GPIO_BSRR_BS0 | GPIO_BSRR_BR1)

// 0: Black active low
// 1: White active low
// 3: Video gate, set = pass, reset = block
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

// There are 52us per visible line, so 52/360 = 144ns per pixel, equivalent to 6.923 MHz (clock by 34.666 for 240MHz input)
// Use timer4, connect to DMA to transfer odsLine -> gpio pins
// We need a 7us delay (plus a bit?) after we detect hsync before starting to output pixels. 1680 ticks @ 240MHz

// How many OSD lines are there to map onto the 625 or 525 video lines? 16 rows of characters x 18 pixels per character = 288 osd lines. Well that's ugly.
// Probably just send them out on both odd and even. Less rows for ntsc.

// Has to be adjusted by hand to compensate for dma startup delays
#define LINE_START_DELAY 1300
#define OSD_FIRST_VISIBLE_LINE 16

// define the character frame buffer for the OSD
#define CHARACTERS_PER_LINE 30
#define NUMBER_OF_TEXT_LINES 16

FAST_DATA uint16_t osdFrame[NUMBER_OF_TEXT_LINES][CHARACTERS_PER_LINE];

#include "font_default.h"

FAST_DATA uint32_t fastFont[FONT_NCHAR][18];

/**
 * Expand the font pixels into a GPIO BRSS word
 * 
 * Used when rendering a line of character data into an output buffer which will be transferred
 * by DMA to the gpio combined set/reset register. The character index is used to look up the pixels
 * in the font table. Each pixel is converted to the corresponding 32 bit brss pattern using the OSD_BLACK,
 * OSD_WHITE and OSD_TRANSPARENT defines and then written into the corresponding location of the output buffer.
 * 
 */
inline void drawCharacterRow(const uint8_t chIndex, const uint16_t osdColumn, const uint8_t rowInCharacter, uint32_t *lineBuffer)
{
  // lookup table to translate from font pixel encoding to OSD gpio words
  static const uint32_t pixelCoding[] = {OSD_BLACK, OSD_TRANSPARENT, OSD_WHITE, OSD_TRANSPARENT};

  // get the character definition from the font table
  const uint32_t *character = fastFont[chIndex];

  // get the row of the character that needs to be rendered
  const uint32_t row = character[rowInCharacter];

  // convert the 12 pixels of the row into the gpio bsrr output format for the osd pixels
  for (uint32_t i=0; i<12; i++)
  {
    const uint8_t pixel = (row >> ((11 - i) * 2)) & 0b11;
    lineBuffer[osdColumn + i] = pixelCoding[pixel];
  }
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
    osdFrame[row][x++] = *(cPtr++);
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


    // if (line > 140 && line < 158)
    // {
    //   // if you don't call this, the HAL_DMA_Start call fails, but it's pretty heavy weight, so it would be good
    //   // to figure out what the minimum action is.
    //   HAL_DMA_Abort(&hdma_tim4_ch1);

    //   HAL_StatusTypeDef status = HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)&(osdTextLines[(line-140)*PIXELS_PER_LINE]), (uint32_t)&(GPIOC->BSRR), PIXELS_PER_LINE);
    //   if (HAL_OK != status) {
    //     Serial.print("dma start failed ");
    //     Serial.println(status);
    //   }
    // // } else if (line > 100 && line < 200)
    // // {
    // //   // Display the test pattern line
    // //
    // //   // if you don't call this, the HAL_DMA_Start call fails, but it's pretty heavy weight, so it would be good
    // //   // to figure out what the minimum action is.
    // //   HAL_DMA_Abort(&hdma_tim4_ch1);

    // //   HAL_StatusTypeDef status = HAL_DMA_Start(&hdma_tim4_ch1, (uint32_t)&(osdLine[0]), (uint32_t)&(GPIOC->BSRR), PIXELS_PER_LINE);
    // //   // HAL_StatusTypeDef status = HAL_DMA_Start(&hdma_tim4_ch1, (uint32_t)&(osdTextLines[9*PIXELS_PER_LINE]), (uint32_t)&(GPIOC->BSRR), PIXELS_PER_LINE);

    // //   if (HAL_OK != status) {
    // //     Serial.print("dma start failed ");
    // //     Serial.println(status);
    // //   }
    // }


    // __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1); // needed?
    // do we need to clear an interrupt bit?


    // hwTimer->resume();

    // This block confirmed that writing to GPIOC->BSRR toggles the gpio in the expected manner
    // if (testBit) {
    //   GPIOC->BSRR = GPIO_BSRR_BS0;
    // } else {
    //   GPIOC->BSRR = GPIO_BSRR_BR0;
    // }
    // testBit = !testBit;

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

// void setLine()
// {
//     // Setup some simple geometric pattern in the output buffer
//   // bit 0 pulls the video darker when set
//   // bit 1 pulls the video lighter when set

//   for(int i=1; i<(PIXELS_PER_LINE-9); ) {

//     // 2 black pixels
//     osdLine[i++] = OSD_BLACK;
//     osdLine[i++] = OSD_BLACK;

//     // 4 white pixels
//     osdLine[i++] = OSD_WHITE;
//     osdLine[i++] = OSD_WHITE;
//     osdLine[i++] = OSD_WHITE;
//     osdLine[i++] = OSD_WHITE;

//     // 2 black pixels
//     osdLine[i++] = OSD_BLACK;
//     osdLine[i++] = OSD_BLACK;

//     // 8 unmodified pixels
//     for (int t=0; t<8; t++) {
//       if (i>=PIXELS_PER_LINE-1) break;  // don't overrun the buffer
//       osdLine[i++] = OSD_TRANSPARENT;
//     }

//     // // divide the line into blocks
//     // // black signal on bit 0
//     // int block = (i-1)/12;
//     // if (block % 2) {
//     //   osdLine[i] = GPIO_BSRR_BS0;
//     // } else {
//     //   osdLine[i] = GPIO_BSRR_BR0;
//     // }
//     // // white signal on bit1
//     // block = (i-1)/24;
//     // if (block % 2) {
//     //   osdLine[i] |= GPIO_BSRR_BR1;
//     // } else {
//     //   osdLine[i] |= GPIO_BSRR_BS1;
//     // }

//   }
//   // The first and last entries in the buffer must always reset the outputs so we don't corrupt the sync pulse
//   // (the first value seems to get sent prematurely. A better fix would be to stop that happening)
//   osdLine[0]                 = OSD_TRANSPARENT;
//   osdLine[PIXELS_PER_LINE-1] = OSD_TRANSPARENT;

// }


// ============================================================================

void setup()
{
    /* Load functions into ITCM RAM */
  extern const unsigned char itcm_text_start;
  extern const unsigned char itcm_text_end;
  extern const unsigned char itcm_data;
  memcpy((void*)&itcm_text_start, &itcm_data, (int) (&itcm_text_end - &itcm_text_start));

  // zero init the osdFrame
  memset(osdFrame, 0, sizeof(osdFrame));

  // copy the font into fast memory
  memcpy(fastFont, font, sizeof(font));

  // setLine();

  delay(100);

  // flush the line out of the data cache
  // #ifndef D_CACHE_DISABLED
  // SCB_CleanDCache_by_Addr((uint32_t*)osdLine, PIXELS_PER_LINE*4);

  // SCB_CleanDCache_by_Addr((uint32_t*)osdTextLines, PIXELS_PER_LINE*4*18);
  // #endif

  // Put some content into the frame buffer
  // osdFrame[8][15] = 'X';
  drawString((const uint8_t*)"HELLO WORLD AGAIN!", 6, 6);

  for(int i=0; i<16; i++) {
    if (i< 10) {
      osdFrame[i][0] = i + '0';
    } else {
        osdFrame[i][0] = i - 10 + 'A';
    }
  }

  for(int i=1; i<30; i++) {
    osdFrame[0][i] = (i % 10) + '0';
  }

  // display the bf logo
  // 4 lines of 24 chars starting at index 160
  const uint8_t xStart = 3, yStart = 2;
  uint8_t cIndex = 160;
  for(int yc = 0; yc < 4; yc++) {
    for (int xc = 0; xc < 24; xc++) {
      osdFrame[yc+yStart][xc+xStart] = cIndex++;
    }
  }


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

  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  __HAL_LINKDMA(&htim4,hdma[TIM_DMA_ID_CC1],hdma_tim4_ch1);


  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1); // required

  // set PA1 for output to the LED
  pinMode(D1, OUTPUT);

  // pinMode(D79, OUTPUT);

  pinMode(D24, OUTPUT); // PA4 which is DAC1_OUT1

  analogWrite(D24, 16); // 0.2V threshold for sync pulses

  Serial.begin(460800);
  Serial.println("Hello!");


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

  // main comp config
  COMP_HandleTypeDef hcomp1;
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

  // start the comparator
  if (HAL_COMP_Start(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  // HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);


  // start the timer
  hwTimer->resume();

}

// is fast code working? It seems to make things slower ?!
FAST_CODE void renderLine()
{
  uint32_t * buffer = osdOutputBuffers[(lineToOutput + 1) % N_OSD_BUFFER_LINES];
  uint16_t lineToRender = line + 2; // may need to store lineToRender in the dma interrupt handler to avoid race conditions
  if (lineToRender < OSD_FIRST_VISIBLE_LINE || lineToRender > (OSD_FIRST_VISIBLE_LINE + 288))
  {
    // need to blank the output buffer
    for(int i=0; i<PIXELS_PER_LINE; i++)
    {
      buffer[i] = OSD_TRANSPARENT;
    }
  } else {
    // figure out which line of the character frame buffer we're looking at
    const uint8_t charLine = (lineToRender - OSD_FIRST_VISIBLE_LINE) / 18;
    const uint8_t rowInCharacter = (lineToRender - OSD_FIRST_VISIBLE_LINE) % 18;

    // for each character in the line
    for (uint8_t i=0; i<30; i++)
    {
      // write the row of the character into the output buffer
      uint8_t charIndex = osdFrame[charLine][i];
      drawCharacterRow(charIndex, i*12, rowInCharacter, buffer);
    }
    // add the special beginning and end of line values
    buffer[0] = OSD_TRANSPARENT;
    buffer[PIXELS_PER_LINE-1] = OSD_TRANSPARENT;
  }

  // force the buffer line out of the data cache
  #ifndef D_CACHE_DISABLED
  SCB_CleanDCache_by_Addr((uint32_t*)buffer, PIXELS_PER_LINE*4);
  #endif

}

#define LED_INTERVAL_MS 500

void loop() {

  static uint32_t ledLastUpdate = 0;
  static uint8_t ledState = 0;

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

  // Render the next line if needed
  if (renderNeeded) {
    uint32_t tStart = micros();
    renderLine();
    renderNeeded = false;
    uint32_t tEnd = micros();
    uint32_t tElapsed = (tEnd -tStart);
    totalRenderTimeMicros += tElapsed;
    nRender++;
    if (tElapsed > renderMax) renderMax = tElapsed;
    if (tElapsed < renderMin) renderMin = tElapsed;
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

