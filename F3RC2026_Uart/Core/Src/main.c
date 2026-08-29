/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile uint8_t Emergencystate=0;//0：正常　1：異常
volatile uint8_t Emergencyreset=2;

volatile uint32_t uart_timeout = 0;
volatile uint8_t uart_received = 0;//0 :未受信 1:受信済み

volatile uint8_t slow = 0; // slowモーション 0:普通 1:スロー
volatile uint8_t slow_count = 0; // slowモーションカウント 0 1

volatile uint8_t stop = 0; // stopモーション 0:普通 1:停止
volatile uint16_t stopCount = 0;

volatile uint32_t recovery_count = 0; // 非常停止中のUART受信カウント（1秒間で90回以上で復帰）
volatile uint32_t recovery_timer = 0; // 非常停止中の1秒ウィンドウ計測用（10ms単位）

uint8_t TxData1[8] = {}; // 足回り基板
uint8_t TxData2[8] = {}; // 回収機構基板

uint8_t buffer[1] = {};
uint8_t size = 1;
int8_t datapos=-1;
uint8_t data[8];
uint16_t FIRSTCANID = 0x300;
uint16_t SECONDCANID = 0x301;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;

//int8⇔uint8変換用
typedef union {
    int8_t i8;
    uint8_t u8;
} int8_uint8_converter;

int8_uint8_converter iuc[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
   return len;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {

        // --- パケット解析処理（正常時・非常停止時ともに共通で実行） ---
        if (datapos == -1) {
            if (buffer[0] == 0xFF) {
                datapos = 0;
            }
        }
        else if (datapos < 8) {
            if (buffer[0] == 0xFF) {
                datapos = 0; // 途中で0xFFが来たらリセットして再スタート
            } else {
                data[datapos] = buffer[0];
                datapos++;
            }
        }
        else if (datapos == 8) {
            if (buffer[0] == 0xFF) { // ★正常なパケットを1つ受信用完了
                
                if (Emergencystate == 0) {
                    // 正常動作時：データを更新してCAN送信
                  if (((data[6] >> 2) & 0x01) && (stopCount == 0)) {
                      stop = !stop;     // 0:通常 ⇔ 1:停止 のトグル切り替え
                      stopCount = 1;    // ガード状態（ボタン連続判定防止）に設定

                      // TIM16を初期化してスタート（チャタリング/長押しガード）
                      HAL_TIM_Base_Stop_IT(&htim16);
                      __HAL_TIM_SET_COUNTER(&htim16, 0);
                      __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
                      HAL_TIM_Base_Start_IT(&htim16);
                  }
                  if (stop == 1) {
                      // 停止状態の時はモータへの指令値を全ゼロにする
                      memset(TxData1, 0, sizeof(TxData1));
                      memset(TxData2, 0, sizeof(TxData2));
                  }else{
                    int8_t val0 = data[3];
                    int8_t val1 = data[5];
                    int8_t val2 = data[2];

                    iuc[0].i8 = (val0 >= -10 && val0 <= 10) ? 0 : val0;
                    iuc[1].i8 = (val1 >= -10 && val1 <= 10) ? 0 : val1;
                    iuc[2].i8 = (val2 >= -10 && val2 <= 10) ? 0 : val2;



                    if (slow == 1) {
                          iuc[0].i8 /= 2;
                          iuc[1].i8 /= 2;
                          iuc[2].i8 /= 2;
                    }
                    if ((data[7] & 0x01) && (slow_count == 0)) {
                      slow = !slow;
                      slow_count = 1;
                      // TIM7のカウント初期化と再スタート
                      HAL_TIM_Base_Stop_IT(&htim7);               // 一旦停止
                      __HAL_TIM_SET_COUNTER(&htim7, 0);          // カウンタリセット
                      __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE); // ★先行してセットされているフラグをクリア
                      HAL_TIM_Base_Start_IT(&htim7);
                    }

                    TxData1[0] = iuc[0].u8;
                    TxData1[1] = iuc[1].u8;
                    TxData1[2] = iuc[2].u8;

                    // data配列がUART等で受信されたデータ（8バイト）だと仮定

                    // 〇（Circle）の値 (data[7] の bit3 に入っているボタン状態: 0 または 1)
                    TxData2[0] = (data[7] >> 3) & 0x01;
                    // □（Square）の値 (data[7] の bit2 に入っているボタン状態: 0 または 1)
                    TxData2[1] = (data[7] >> 2) & 0x01;
                    // R1 (ボタン状態: data[7] の bit5) -> TxData2[2]
                    TxData2[2] = (data[7] >> 5) & 0x01;
                    // R2 (アナログ値 0〜255: data[0]) -> TxData2[3]
                    if (data[0] > 60)
                        TxData2[3] = 1;
                    else
                        TxData2[3] = 0;
                    // L1 (ボタン状態: data[7] の bit4) -> TxData2[4]
                    TxData2[4] = (data[7] >> 4) & 0x01;
                    // L2 (アナログ値 0〜255: data[1]) -> TxData2[5]
                    if (data[1] > 60)
                        TxData2[5] = 1;
                    else
                        TxData2[5] = 0;
                    // printf("TxData1[0]:%d\r\n", TxData1[0]);
                    // printf("TxData1[1]:%d\r\n", TxData1[1]);
                    // printf("TxData1[2]:%d\r\n", TxData1[2]);
                    // printf("TxData1[3]:%d\r\n", TxData1[3]);
                    // printf("TxData1[4]:%d\r\n", TxData1[4]);
                    // printf("TxData1[5]:%d\r\n", TxData1[5]);
                    // printf("TxData1[6]:%d\r\n", TxData1[6]);
                    // printf("TxData1[7]:%d\r\n", TxData1[7]);
                    // printf("TxData2[0]:%d\r\n", TxData2[0]);
                    // printf("TxData2[1]:%d\r\n", TxData2[1]);
                    // printf("TxData2[2]:%d\r\n", TxData2[2]);
                    // printf("TxData2[3]:%d\r\n", TxData2[3]);
                    // printf("TxData2[4]:%d\r\n", TxData2[4]);
                    // printf("TxData2[5]:%d\r\n", TxData2[5]);
                    // printf("TxData2[6]:%d\r\n", TxData2[6]);
                    // printf("TxData2[7]:%d\r\n", TxData2[7]);
                    uart_received = 1;
                    uart_timeout = 0;

                    TxHeader.Identifier = 0x105;
                    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData1);
                    TxHeader.Identifier = 0x205;
                    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData2);
                } 
              }
                else if (Emergencystate == 1) {
                    // 非常停止時：正常パケット数をカウント
                    recovery_count++;
                }

                datapos = 0; // 次のパケット開始用
            } 
            else {
                datapos = -1; // 不正パケット破棄
            }
        }

        // 次の1バイトを受信
        HAL_UART_Receive_IT(&huart1, buffer, size);
}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // エラーフラグ（ORE: オーバーラン, NE: ノイズ, FE: フレーム, PE: パリティ）をクリア
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);

        // 受信データレジスタのゴミデータを空読みしてフラグを確実に解除
        volatile uint32_t dummy = huart->Instance->RDR;
        (void)dummy;

        datapos = -1;

        // 受信割り込みを再開
        HAL_UART_Receive_IT(&huart1, buffer, size);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  setbuf(stdout, NULL);
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
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  // デバッグ停止中（ブレークポイント時）にタイマがカウントアップして割り込みが乱発するのを防ぐ
  __HAL_DBGMCU_FREEZE_TIM6();
  __HAL_DBGMCU_FREEZE_TIM7();
  __HAL_DBGMCU_FREEZE_TIM16();

  // UART初期化時の残留エラーフラグをクリア
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
  HAL_UART_Receive_IT(&huart1, buffer, size);
  HAL_TIM_Base_Start_IT(&htim6);

  TxHeader.Identifier = 0x105;                 // メッセージID (0x105)
  TxHeader.IdType = FDCAN_STANDARD_ID;         // 標準ID (11ビット)
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;     // データフレーム
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;     // 8バイト送信
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;      // ビットレート切り替えなし
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;       // ※通常のCAN通信の場合 (FD通信なら FDCAN_FD_CAN にします)
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  // 2. FDCAN通信の開始
  HAL_FDCAN_Start(&hfdcan1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(1000);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 3000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ouput_GPIO_Port, LED_ouput_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_ouput_Pin */
  GPIO_InitStruct.Pin = LED_ouput_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ouput_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        if (Emergencystate == 0)
        {
            // 正常時：タイムアウト検出
            if (uart_received == 1)
            {
                uart_timeout++;

                // UARTから約1秒データが来なかった → 非常停止
                // TIM6は10ms周期なので、100回 = 1秒
                if (uart_timeout >= 100)
                {
                    Emergencystate = 1;
                    recovery_count = 0; // 復帰カウンタリセット
                    recovery_timer = 0; // 1秒ウィンドウタイマーリセット
                    datapos = -1;       // パケット受信状態リセット

                    // モータ停止用ゼロデータをCAN送信
                    for (int i = 0; i < 8; i++)
                    {
                        TxData1[i] = 0;
                        TxData2[i] = 0;
                    }
                    TxHeader.Identifier = 0x105;
                    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData1);
                    TxHeader.Identifier = 0x205;
                    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData2);
                }
            }
        }
        else if (Emergencystate == 1)
        {
            // 非常停止中：1秒ウィンドウで復帰判定
            recovery_timer++;

            if (recovery_timer >= 100) // 100回 × 10ms = 1秒経過
            {
                if (recovery_count >= 90)
                {
                    // 1秒間に90回以上UART受信 → 復帰
                    Emergencystate = 0;
                    uart_timeout = 0;
                    uart_received = 0;
                    datapos = -1;
                    recovery_count = 0;
                    recovery_timer = 0;
                }
                else
                {
                    // 90回未満 → 非常停止継続、カウンタリセットして次の1秒を計測
                    recovery_count = 0;
                    recovery_timer = 0;

                    // 非常停止中もモータ停止用ゼロデータを定期送信
                    for (int i = 0; i < 8; i++)
                    {
                        TxData1[i] = 0;
                        TxData2[i] = 0;
                    }
                    TxHeader.Identifier = 0x105;
                    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData1);
                    TxHeader.Identifier = 0x205;
                    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData2);
                }
            }
        }
    }
  else if (htim->Instance == TIM7)
  {
    slow_count = 0;
    // 指定時間経過したらタイマーを停止し、切替受付を解禁
    HAL_TIM_Base_Stop_IT(&htim7);
  
  }
  else if (htim->Instance == TIM16)
  {
    stopCount = 0;
    // 指定時間経過したらタイマーを停止し、切替受付を解禁
    HAL_TIM_Base_Stop_IT(&htim16);
  }
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
