/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_audio.h"
#include "stdio.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	CTRL_MODE_ENABLE,
	CTRL_MODE_PAUSE
} CTRL_MODE_Typedef;

typedef struct{
	uint8_t Touched;
	uint16_t x;
	uint16_t y;
} TouchPoint2D_Typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_SCREEN_HORIZONTAL_CENTER 133
#define LCD_SCREEN_X_SIZE 480
#define LCD_SCREEN_Y_SIZE 272
#define LCD_SCREEN_FOREGROUND 1
#define LCD_SCREEN_BACKGROUND 0
#define LCD_COLOR_TOTAL_TRANSPARENT 0x00000000

#define FFT_DATA_LENGTH 4096
#define FFT_TYPE_RFFT 0
#define FFT_TYPE_RIFFT 1

#define AUDIO_SAMPLE_FREQUENCY AUDIO_FREQUENCY_8K
#define AUDIO_RECORD_LENGTH LCD_SCREEN_X_SIZE*4
#define AUDIO_CHANNEL_RIGHT 0
#define AUDIO_CHANNEL_LEFT 2
#define AUDIO_SELECT_CHANNEL AUDIO_CHANNEL_LEFT

#define TIME_DOMAIN_DISPLAY_LIMIT 65
#define TIME_DOMAIN_AMPLITUDE_SCALE 5
#define TIME_DOMAIN_AMPLITUDE_LIMIT TIME_DOMAIN_DISPLAY_LIMIT*TIME_DOMAIN_AMPLITUDE_SCALE

#define FREQUENCY_DOMAIN_DISPLAY_LIMIT 130
#define FREQUENCY_DOMAIN_AMPLITUDE_SCALE 400
#define FREQUENCY_DOMAIN_AMPLITUDE_LIMIT FREQUENCY_DOMAIN_DISPLAY_LIMIT*FREQUENCY_DOMAIN_AMPLITUDE_SCALE

#define CURSOR_SIZE 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai2_a;
DMA_HandleTypeDef hdma_sai2_b;

UART_HandleTypeDef huart1;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
// FFT
float32_t FFT_TimeInput[FFT_DATA_LENGTH] = {0};
float32_t FFT_FrequencyOutput[FFT_DATA_LENGTH*2] = {0};
arm_rfft_fast_instance_f32 FFT_Instance;
uint16_t fftSize = FFT_DATA_LENGTH;
uint8_t ifftFlag = FFT_TYPE_RFFT;
// Audio
uint16_t AudioRecordBuffer[AUDIO_RECORD_LENGTH*2] = {0};
int16_t TimeDomainData[FFT_DATA_LENGTH] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// LCD initialize
void LCD_Init(void) {
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+0x00200000); 
	
	// front 
	BSP_LCD_SelectLayer(LCD_SCREEN_FOREGROUND);
	BSP_LCD_SetBackColor(LCD_COLOR_TOTAL_TRANSPARENT);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_Clear(LCD_COLOR_TOTAL_TRANSPARENT);
	
	// back
	BSP_LCD_SelectLayer(LCD_SCREEN_BACKGROUND);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	
}

// LCD diplay Error message
void LCD_DisplayError(char *Message) {
	BSP_LCD_SelectLayer(LCD_SCREEN_FOREGROUND);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t *)"ERROR", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80, (uint8_t *)Message, CENTER_MODE);
}

// touch screen Initialize
void TS_Init() {
	uint8_t status;
	
	status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	if (status != TS_OK)
  {
		LCD_DisplayError("Touchscreen cannot be initialized");
		while(1);
  }
}

// Audio Initialize
void Audio_Init() {
	uint8_t status;
	
	status = BSP_AUDIO_IN_Init(AUDIO_SAMPLE_FREQUENCY, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
	if (status != AUDIO_OK)
  {
		LCD_DisplayError("Audio cannot be initialized");
		while(1);
  }
}

// FFT initialize
void FFT_Init(arm_rfft_fast_instance_f32 *s, uint16_t fftSize) {
	arm_status fft_res = ARM_MATH_ARGUMENT_ERROR;
	
	fft_res = arm_rfft_fast_init_f32(s, fftSize);
	if (fft_res != ARM_MATH_SUCCESS){
		LCD_DisplayError("FFT cannot be initialized");
		while(1);
	}
}

// Display Center Line
void DisplayAxis() {
	BSP_LCD_SelectLayer(LCD_SCREEN_FOREGROUND);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DrawHLine(0, LCD_SCREEN_HORIZONTAL_CENTER, BSP_LCD_GetXSize());
	BSP_LCD_DrawHLine(0, LCD_SCREEN_HORIZONTAL_CENTER + 1, BSP_LCD_GetXSize());
	BSP_LCD_DrawHLine(0, LCD_SCREEN_HORIZONTAL_CENTER + 2, BSP_LCD_GetXSize());
}

// Display Legend
void DisplayLegend(uint32_t Frequency, uint32_t Power) {
	char text[20];
	// Legend for Frequency and Power
	BSP_LCD_SelectLayer(LCD_SCREEN_FOREGROUND);
	BSP_LCD_SetBackColor(LCD_COLOR_TOTAL_TRANSPARENT);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_SetFont(&Font16);
	sprintf(text,"Frequency:%5d", Frequency);
	BSP_LCD_DisplayStringAt(300, 150, (uint8_t*)text, LEFT_MODE);
	sprintf(text,"Power:%9d", Power);
	BSP_LCD_DisplayStringAt(300, 170, (uint8_t*)text, LEFT_MODE);
}

// time domain display Data process
void TimeDomainDataPreprocess(int16_t *Data, int16_t *DisplayPoint) {
	uint16_t i;
	
	for (i = 0; i < LCD_SCREEN_X_SIZE; i++){
		// display limit
		if (Data[i] >= TIME_DOMAIN_AMPLITUDE_LIMIT)
			DisplayPoint[i] = TIME_DOMAIN_AMPLITUDE_LIMIT/TIME_DOMAIN_AMPLITUDE_SCALE;
		else if (Data[i] <= -TIME_DOMAIN_AMPLITUDE_LIMIT)
			DisplayPoint[i] = -TIME_DOMAIN_AMPLITUDE_LIMIT/TIME_DOMAIN_AMPLITUDE_SCALE;
		else 
			DisplayPoint[i] = Data[i]/TIME_DOMAIN_AMPLITUDE_SCALE;
	}
}

// Display Time Domain
void DisplayTimeDomainData(int16_t *Data) {
	const uint16_t TimeDisplayBase = 65;
	uint16_t i;
	int16_t DisplayPoint[LCD_SCREEN_X_SIZE];
	
	TimeDomainDataPreprocess(Data, DisplayPoint);
	BSP_LCD_SelectLayer(LCD_SCREEN_BACKGROUND);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawVLine(0, 0, LCD_SCREEN_HORIZONTAL_CENTER);
	for (i = 1; i < LCD_SCREEN_X_SIZE; i++){
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawVLine(i, 0, LCD_SCREEN_HORIZONTAL_CENTER);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DrawLine(i-1, TimeDisplayBase-DisplayPoint[i-1], i, TimeDisplayBase-DisplayPoint[i]);
	}
}

// Find Average Value
float32_t FindAverageValue(float32_t *Data, uint16_t CompareSize) {
	uint16_t i;
	float32_t Sum = *Data;
	
	for (i = 1; i < CompareSize; i++){
		Sum += *(Data+i);
	}
	return Sum/CompareSize;
}

// frequency domain display Data process
void FrequencyDomainDataPreprocess(float32_t *Data, uint16_t *DisplayPoint) {
	uint16_t i;
	float32_t CompareSize = (float32_t)FFT_DATA_LENGTH/2/LCD_SCREEN_X_SIZE;
	float32_t ProcessingData[LCD_SCREEN_X_SIZE] = {0};
	
	for (i = 0; i < LCD_SCREEN_X_SIZE; i++){
		ProcessingData[i] = FindAverageValue(&Data[(uint16_t)(CompareSize*i)], (uint16_t)CompareSize);
	}
	
	for (i = 0; i < LCD_SCREEN_X_SIZE; i++){
		// display limit
		if (ProcessingData[i] >= FREQUENCY_DOMAIN_AMPLITUDE_LIMIT)
			DisplayPoint[i] = (uint16_t)(FREQUENCY_DOMAIN_AMPLITUDE_LIMIT/FREQUENCY_DOMAIN_AMPLITUDE_SCALE);
		else 
			DisplayPoint[i] = (uint16_t)(ProcessingData[i]/FREQUENCY_DOMAIN_AMPLITUDE_SCALE);
	}
}

// Display Frequency Domain
void DisplayFrequencyDomainData(float32_t *Data) {
	const uint16_t FrequencyDisplayBase = 270;
	uint16_t i;
	uint16_t DisplayPoint[LCD_SCREEN_X_SIZE];
	
	FrequencyDomainDataPreprocess(Data, DisplayPoint);
	BSP_LCD_SelectLayer(LCD_SCREEN_BACKGROUND);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawVLine(0, LCD_SCREEN_HORIZONTAL_CENTER+3, 138);
	for (i = 1; i < LCD_SCREEN_X_SIZE; i++){
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawVLine(i, LCD_SCREEN_HORIZONTAL_CENTER+3, 138);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DrawLine(i-1, FrequencyDisplayBase - DisplayPoint[i-1], i, FrequencyDisplayBase-DisplayPoint[i]);
	}
}

// Display Pause
void DisplayPause(void) {	
	BSP_LCD_SelectLayer(LCD_SCREEN_FOREGROUND);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DisplayStringAt(0, 30,(uint8_t*)"pause",CENTER_MODE);
}

// Audio Callback Function
void AUDIO_IN_CallBack(void) {
	uint16_t i;
	memcpy(&TimeDomainData[LCD_SCREEN_X_SIZE], TimeDomainData, (FFT_DATA_LENGTH-LCD_SCREEN_X_SIZE)*sizeof(int16_t));
	for(i = 0; i < LCD_SCREEN_X_SIZE; i++){
		TimeDomainData[i] = AudioRecordBuffer[i*4 + AUDIO_SELECT_CHANNEL];
	}
	for (i = 0; i < FFT_DATA_LENGTH; i++){
		FFT_TimeInput[i] = (float32_t)TimeDomainData[i];
	}
	arm_rfft_fast_f32(&FFT_Instance, FFT_TimeInput, FFT_FrequencyOutput, ifftFlag);
	arm_cmplx_mag_f32(FFT_FrequencyOutput, FFT_FrequencyOutput, FFT_DATA_LENGTH*2);
	BSP_AUDIO_IN_Record(AudioRecordBuffer, AUDIO_RECORD_LENGTH);
}

// whether the touch point under line
uint8_t CheckTouchPointUnderReferenceLine(uint16_t *PointArray, uint8_t DetectedPoint, uint16_t ReferenceLine) {
	uint8_t i;
	for (i = 0; i < DetectedPoint; i++){
		if (*(PointArray + i) <= ReferenceLine)
			return i+1;
	}
	return 0;
}

// whether the touch point above line
uint8_t CheckTouchPointAboveReferenceLine(uint16_t *PointArray, uint8_t DetectedPoint, uint16_t ReferenceLine) {
	uint8_t i;
	for (i = 0; i < DetectedPoint; i++){
		if (*(PointArray + i) > ReferenceLine)
			return i+1;
	}
	return 0;
}

// convert x axis to Frquency
uint32_t ConvertXAxisToFrequency(uint16_t Point) {
	return Point*AUDIO_SAMPLE_FREQUENCY/2/LCD_SCREEN_X_SIZE;
}

// Use X axis point to Get Power 
uint32_t GetPowerWithXAxis(float32_t *Data, uint16_t Point) {
	float32_t CompareSize = (float32_t)FFT_DATA_LENGTH/2/LCD_SCREEN_X_SIZE;
	float32_t Value = FindAverageValue(&Data[(uint16_t)(CompareSize*Point)], (uint16_t)CompareSize); 
	return (uint32_t)Value;
}

// Draw Cursor
void DrawCursor(uint16_t x, uint16_t y) {
	static uint16_t OldCursorX, OldCursorY, OldCursorWidth, OldCursorHeight;
	uint16_t XPos = x-CURSOR_SIZE/2, YPos = y-CURSOR_SIZE/2, Width = CURSOR_SIZE, Height = CURSOR_SIZE;
	if (x <= CURSOR_SIZE/2) {
		XPos = 0;
		Width = CURSOR_SIZE/2;
	}
	if (x > LCD_SCREEN_X_SIZE-CURSOR_SIZE/2){
		XPos = LCD_SCREEN_X_SIZE-CURSOR_SIZE/2;
		Width = CURSOR_SIZE/2;
	}
	if (y > LCD_SCREEN_Y_SIZE-CURSOR_SIZE/2){
		YPos = LCD_SCREEN_Y_SIZE-CURSOR_SIZE/2;
		Height = CURSOR_SIZE/2;
	}
	// clean Old Cursor
	BSP_LCD_SetTextColor(LCD_COLOR_TOTAL_TRANSPARENT);
	BSP_LCD_FillRect(OldCursorX, OldCursorY, OldCursorWidth, OldCursorHeight);
	OldCursorX = XPos;
	OldCursorY = YPos;
	OldCursorWidth = Width;
	OldCursorHeight = Height;
	// New Cursor
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_FillRect(XPos, YPos, Width, Height);
}

// Display FrequencyDomain Cursor
void DisplayFrequencyCursor(uint16_t x) {
	uint32_t DisplayPoint = GetPowerWithXAxis(FFT_FrequencyOutput, x);
	
	DisplayPoint = (DisplayPoint/FREQUENCY_DOMAIN_AMPLITUDE_SCALE > FREQUENCY_DOMAIN_DISPLAY_LIMIT) ? FREQUENCY_DOMAIN_DISPLAY_LIMIT :
								  (uint16_t)DisplayPoint/FREQUENCY_DOMAIN_AMPLITUDE_SCALE;
	DrawCursor(x, 270 - DisplayPoint);
}

// Clean Up Part of foreground
void LCD_CleanForegroundUp(void) {
	BSP_LCD_SelectLayer(LCD_SCREEN_FOREGROUND);
	BSP_LCD_SetTextColor(LCD_COLOR_TOTAL_TRANSPARENT);
	BSP_LCD_FillRect(0, 0, LCD_SCREEN_X_SIZE, LCD_SCREEN_HORIZONTAL_CENTER);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	TS_StateTypeDef  TS_State;
	CTRL_MODE_Typedef CtrlMode = CTRL_MODE_ENABLE;
	TouchPoint2D_Typedef TouchedPoint = {0, 0, 0};
	uint8_t AboveTouchPointIndex;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	TS_Init();
	Audio_Init();
	FFT_Init(&FFT_Instance, fftSize);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	DisplayAxis();
	DisplayLegend(0, 0);

	BSP_AUDIO_IN_Record(AudioRecordBuffer, AUDIO_RECORD_LENGTH);
  while (1)
  {
		// Touch Screen
		BSP_TS_GetState(&TS_State);
		
		if (TS_State.touchDetected > 0){
			// Time Domain Touch
			if (CheckTouchPointUnderReferenceLine(TS_State.touchY, TS_State.touchDetected, LCD_SCREEN_HORIZONTAL_CENTER) > 0){
				// Detect First Touch
				if (TouchedPoint.Touched == 0)
					CtrlMode = (CtrlMode == CTRL_MODE_ENABLE) ? CTRL_MODE_PAUSE : CTRL_MODE_ENABLE;
				// operation for different mode
				if (CtrlMode == CTRL_MODE_ENABLE){
					BSP_AUDIO_IN_Record(AudioRecordBuffer, AUDIO_RECORD_LENGTH);
					LCD_CleanForegroundUp();
				}
				else {
					BSP_AUDIO_IN_Stop(CODEC_PDWN_SW);
					DisplayPause();
				}
			}
			// Frequency Domain Touch
			AboveTouchPointIndex = CheckTouchPointAboveReferenceLine(TS_State.touchY, TS_State.touchDetected, LCD_SCREEN_HORIZONTAL_CENTER);
			if(AboveTouchPointIndex > 0) {
				TouchedPoint.x = TS_State.touchX[AboveTouchPointIndex-1];
				TouchedPoint.y = TS_State.touchY[AboveTouchPointIndex-1];
			}
			TouchedPoint.Touched = TS_State.touchDetected;
		}
		else { // clean touch state
			TouchedPoint.Touched = 0;
		}
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
		if (CtrlMode == CTRL_MODE_ENABLE){
			DisplayTimeDomainData(TimeDomainData);
		}
		DisplayFrequencyDomainData(FFT_FrequencyOutput);
		DisplayLegend(ConvertXAxisToFrequency(TouchedPoint.x), GetPowerWithXAxis(FFT_FrequencyOutput, TouchedPoint.x));
		DisplayFrequencyCursor(TouchedPoint.x);
		HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_SAI2;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PG14 PG13 PG11 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB13 PB12 PB10 
                           PB1 PB0 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_13|GPIO_PIN_12|GPIO_PIN_10 
                          |GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC12 PC11 PC10 PC9 
                           PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PJ12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA11 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PI3 PI2 PI12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PK3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : PG9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PH15 PH2 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PI1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PH13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PH14 PH12 PH9 PH11 
                           PH10 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_12|GPIO_PIN_9|GPIO_PIN_11 
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PI0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PI13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PH4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PG7 PG6 PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF6 PF10 PF9 
                           PF8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_9 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PG2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PH7 PH8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PH6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
