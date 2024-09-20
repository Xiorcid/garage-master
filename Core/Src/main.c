/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <dispcolor.h>
#include <gpio.h>
#include "info_disp.h"
#include "buttons.h"
#include "enc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATE_INIT    0
#define STATE_DISPLAY 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SCROLL_MODE_AUTO  0
#define SCROLL_MODE_HALT  1
#define SCROLL_MODE_CLICK 2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
// DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart8;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_uart8_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
#define DEV0_UART &huart5


#define DEV_STATE_IN    1
#define DEV_STATE_UNIN  0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_UART5_Init(void);
static void MX_UART8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double val = 10;
uint8_t tx_buff[]={84,66,67,68,69,70,71,72,73,74};
uint8_t palette = TEMP_PALETTE;
uint8_t device_type = TYPE_SET_TEL;

uint8_t state = STATE_INIT;
uint8_t dev_num = 0;

uint8_t screen_scroll_mode = SCROLL_MODE_AUTO;

Button button = {GPIOC, GPIO_PIN_13, TYPE_HIGH_PULL};

Encoder enc = {GPIOD, GPIO_PIN_0, GPIOD, GPIO_PIN_1};

// Device device_0 = {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY};
// Device device_2 = {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY};
// Device device_3 = {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY};
// Device device_4 = {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY};

Device deviceList[] = {{&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY}};

#define DEV_COUNT sizeof(deviceList)/sizeof(deviceList[0])
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	


	gpio_SetGPIOmode_Out(GPIOB, GPIO_PIN_14);
  gpio_SetGPIOmode_Out(GPIOB, GPIO_PIN_7);
  gpio_SetGPIOmode_Out(GPIOB, GPIO_PIN_0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(100);


	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_UART5_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
  init(&button);
  initEnc(&enc);
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	dispcolor_Init(240, 240);

	uint32_t dev_init_timeout = HAL_GetTick();
  uint32_t screen_disp_time = HAL_GetTick();
  uint32_t hold_timeout = HAL_GetTick();

  uint32_t simtmr = HAL_GetTick();

	bool isOn = 0;
  
  uint8_t old_dev_num = 0;

  uint8_t rx_buff[5];

  bool testMode = MODE_NORMAL;
  uint16_t testVal = 28.5;

  deviceList[0].tx_buff[0] = 84;

  //Show_Message("HELLO, WORLD!", 5000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // NEW CODE START
    if(state == STATE_INIT){
      Display_Init(dev_num, DEV_COUNT);
      HAL_UART_Transmit_DMA(deviceList[dev_num].uart, deviceList[dev_num].tx_buff, 10);
      HAL_UART_Receive_DMA(deviceList[dev_num].uart, deviceList[dev_num].rx_buff, 10);
    }else{
      Dispaly_Data(&deviceList[dev_num]);
      HAL_UART_Transmit_DMA(deviceList[dev_num].uart, deviceList[dev_num].tx_buff, 10);
    }
    // NEW CODE END

    // DEVICE INITIALIZATION START
    if (HAL_GetTick()-dev_init_timeout>50 && state == STATE_INIT){
      if(dev_num == old_dev_num){
        dev_num++;
      }else{
        old_dev_num = dev_num;
      }
      dev_init_timeout = HAL_GetTick();
      screen_disp_time = HAL_GetTick();
    }

    if(dev_num == DEV_COUNT && state == STATE_INIT){
      state = STATE_DISPLAY;
      dev_num = 0;
    }
    // DEVICE INITIALIZATION END

    // AUTO SCREEN SCROLL START
    if (HAL_GetTick()-screen_disp_time>5000 && screen_scroll_mode == SCROLL_MODE_AUTO){
      if(state != STATE_INIT){
        do{
          if(dev_num < DEV_COUNT-1){
            dev_num++;
          }else{
            dev_num=0;
          }
        }while (!deviceList[dev_num].initState);  
      }
      screen_disp_time = HAL_GetTick();
    }
    // AUTO SCREEN SCROLL END

    // GPIO READING START
    tick(&button);
    tickEnc(&enc);
    // GPIO READING END

    // SCROLL MODE CHANGE START
    if(isClicked(&button)){
      screen_disp_time = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
      if(screen_scroll_mode == SCROLL_MODE_AUTO){
        screen_scroll_mode = SCROLL_MODE_HALT;
      }else{
        screen_scroll_mode = SCROLL_MODE_AUTO;
      }
    }
    // SCROLL MODE CHANGE END

    // ENCODER START
    // TEST CODE START
    if (deviceList[dev_num].deviceDisplayMode == MODE_NORMAL){
    // TEST CODE
      // MANUAL SCREEN SROLL START
      if(isRight(&enc)){
        if(dev_num < DEV_COUNT){
          dev_num++;
        }else{
          dev_num = 0;
        }
        screen_disp_time = HAL_GetTick();
      }

      if(isLeft(&enc)){
        if(dev_num > 0){
          dev_num--;
        }else{
          dev_num = DEV_COUNT;
        }
        screen_disp_time = HAL_GetTick();
      }
      // MANUAL SCREEN SCROLL START
    }else{
      // VALUE EDIT START
      if(isRight(&enc)){
        deviceList[dev_num].setValue +=10;
      }
      if(isLeft(&enc)){
        deviceList[dev_num].setValue -=10;
      }
      // VALUE EDIT END
    }
    // ENCODER END

    // EDIT VALUE START
    if(isHold(&button) && HAL_GetTick() - hold_timeout > 750){
      screen_disp_time = HAL_GetTick();
      hold_timeout = HAL_GetTick();
      if(deviceList[dev_num].deviceDisplayMode == MODE_NORMAL){
        deviceList[dev_num].setValue = deviceList[dev_num].currentValue;
        deviceList[dev_num].deviceDisplayMode = MODE_EDIT;
      }else{
        char setValueTx[10];
        sprintf(setValueTx, "%d", deviceList[dev_num].setValue);
        HAL_UART_Transmit_DMA(deviceList[dev_num].uart, setValueTx, 10);
        deviceList[dev_num].deviceDisplayMode = MODE_NORMAL;
      }

      if(screen_scroll_mode == SCROLL_MODE_AUTO){
        screen_scroll_mode = SCROLL_MODE_HALT;
      }else{
        screen_scroll_mode = SCROLL_MODE_AUTO;
      }
    }
    // EDIT VALUE END

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DC_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin RST_Pin */
  GPIO_InitStruct.Pin = DC_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_S1_Pin ENC_S2_Pin */
  GPIO_InitStruct.Pin = ENC_S1_Pin|ENC_S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /*
  COMMUNICATION PROTOCOL
  Master    Slave
  T      -> palette(L/T), type(S/T/B), I(value), X(value), (symbol) ... LBI0X250%
  G      -> value ... 754
  val    -> OK
  ON     -> OK
  OFF    -> OK
  */

  HAL_UART_Receive_DMA(deviceList[dev_num].uart, deviceList[dev_num].rx_buff, 10);

    // NEW CODE START
    // IF FIRST LETTER IS L OR T
  if(deviceList[dev_num].rx_buff[0] == 76 || deviceList[dev_num].rx_buff[0] == 84){
    // IF L
    if (deviceList[dev_num].rx_buff[0] == 76){
      deviceList[dev_num].paletteType = LIGHT_PALETTE;
    }else{
      deviceList[dev_num].paletteType = TEMP_PALETTE;
    }
    // ENDIF

    // DEVICE MODE
    switch (deviceList[dev_num].rx_buff[1])
    {
    case 83:
      deviceList[dev_num].deviceMode = TYPE_SET_ONLY;
      break;
    case 84:
      deviceList[dev_num].deviceMode = TYPE_TEL_ONLY;
      break;
    case 66:
      deviceList[dev_num].deviceMode = TYPE_SET_TEL;
      break;
    }
    // ENDIF

    uint8_t minValueStartIndex;
    uint8_t minValueEndIndex;

    for (int i = 2; i<sizeof(deviceList[dev_num].rx_buff); i++){
      if(deviceList[dev_num].rx_buff[i] == 73){
        minValueStartIndex = i+1;
        continue;
      }
      if(deviceList[dev_num].rx_buff[i] == 88){
        minValueEndIndex = i-1;
        break;
      }
    }

    uint8_t minValueArr[minValueEndIndex-minValueStartIndex];
    uint8_t maxValueArr[sizeof(deviceList[dev_num].rx_buff)-minValueEndIndex-1];

    for (int i = minValueStartIndex; i<minValueEndIndex+1; i++){
      minValueArr[i-minValueStartIndex] = deviceList[dev_num].rx_buff[i];
    }

    for (int i = minValueEndIndex+2; i<sizeof(deviceList[dev_num].rx_buff)-1; i++){
      minValueArr[i-minValueEndIndex] = deviceList[dev_num].rx_buff[i];
    }

    deviceList[dev_num].minValue = atof(minValueArr);
    deviceList[dev_num].maxValue = atof(maxValueArr);

    deviceList[dev_num].symbol = deviceList[dev_num].rx_buff[sizeof(deviceList[dev_num].rx_buff)];


    deviceList[dev_num].tx_buff[0] = 71;
  }else if (deviceList[dev_num].rx_buff[0] == 79){
    //PASS
  }else{
    deviceList[dev_num].currentValue = atof(deviceList[dev_num].rx_buff);
    deviceList[dev_num].currentValue /= 10;
  }
  // NEW CODE END

  if(state == STATE_INIT){
    deviceList[dev_num].initState = DEV_STATE_IN;
    dev_num++;
  }

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  Show_Message("Kernel panic", 60000);
  NVIC_SystemReset();
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
