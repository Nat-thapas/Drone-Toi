/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bmp280.h"
#include "bno055.h"
#include "bno055_stm32.h"
#include "float.h"
#include "math.h"
#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TELEMETRY_INTERVAL 250000  // us.

#define RADIO_RX_UART huart1
#define RADIO_TELEM_UART huart2
#define SERIAL_UART huart3
#define WLSER_UART huart6

#define PRECISION_TIMER_TIM htim2
#define MOTORS_PWM_TIM htim3

#define IMU_I2C hi2c1
#define ALT_I2C hi2c2
#define TOF_I2C hi2c3

#define PRECISION_TIMER_COUNT htim2.Instance->CNT

// After threshold error 1 byte will be skipped to try to align incoming data
#define RADIO_CONSECUTIVE_ERROR_THRESHOLD 3

// Value that is within +- deadzone of middle (1500) will be set to middle (1500)
// only active on channels that return to center (1, 2, 4)
#define RADIO_DEADZONE 25

// Gain units are roughly in % power increase/decrease per degree error
#define PID_PROPORTIONAL_GAIN 1.f
#define PID_INTEGRAL_GAIN 0.f
#define PID_DERIVATIVE_GAIN 0.f

#define I2C_PERIPHERAL_INIT_RETRY_LIMIT 5

#define MOTOR_1_FL 1
#define MOTOR_2_RR 2
#define MOTOR_3_FR 3
#define MOTOR_4_RL 4

#define MOTOR_1_FL_PWM_CCR htim3.Instance->CCR1
#define MOTOR_2_RR_PWM_CCR htim3.Instance->CCR4
#define MOTOR_3_FR_PWM_CCR htim3.Instance->CCR2
#define MOTOR_4_RL_PWM_CCR htim3.Instance->CCR3

#define BATT_SENSOR_ADC hadc1
#define BATT_VOLTAGE_SCALE 3.3f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined(__ICCARM__) /*!< IAR Compiler */
#pragma location = 0x2007c000
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location = 0x2007c0a0
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined(__CC_ARM) /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined(__GNUC__) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]
    __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]
    __attribute__((section(".TxDecripSection"))); /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_i2c4_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

int_fast32_t precisionTimer_acc_us = 0;

int_fast16_t radio_consecutiveErrors = 0;

int_fast32_t telemetry_lastTransmission_us = 0;
int_fast16_t telemetry_iterCount = 0;

uint8_t radio_rxBuffer[32] = {};
int_fast16_t radio_rxValues[10] = {
    1000,
    1000,
    1000,
    1000,
    1000,
    1000,
    1000,
    1000,
    1000,
    1000,
};
const float radio_targetAngleLimits[3] = {5.f, 10.f, 30.f};

uint32_t batt_adcRawValue = 0;

int_fast32_t pid_lastLoopTime_us = 0;
float pid_lastRollError = NAN;
float pid_lastPitchError = NAN;
float pid_cumulativeRollError = 0.f;
float pid_cumulativePitchError = 0.f;

BMP280_HandleTypedef bmp280;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int_fast32_t GetTickPrecise() {
  return precisionTimer_acc_us + PRECISION_TIMER_COUNT;
}

void UART_Transmit_DMA(UART_HandleTypeDef *huart, char *str) {
  HAL_UART_Transmit_DMA(huart, (uint8_t *)str, strlen(str));
}

void UART_Transmitf_DMA(UART_HandleTypeDef *huart, const char *format, ...) {
  char buffer[2048];  // Buffer to hold formatted string
  va_list args;

  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  UART_Transmit_DMA(huart, buffer);
}

void UART_Transmitln_DMA(UART_HandleTypeDef *huart, char *str) {
  UART_Transmitf_DMA(huart, "%s\r\n", str);
}

void motors_SetPower(uint8_t motor, float power) {
  if (power < 0.f) power = 0.f;
  if (power > 1.f) power = 1.f;
  uint16_t value = power * 1000.f + 1000.f;
  switch (motor) {
    case MOTOR_1_FL:
      MOTOR_1_FL_PWM_CCR = value;
      break;
    case MOTOR_2_RR:
      MOTOR_2_RR_PWM_CCR = value;
      break;
    case MOTOR_3_FR:
      MOTOR_3_FR_PWM_CCR = value;
      break;
    case MOTOR_4_RL:
      MOTOR_4_RL_PWM_CCR = value;
      break;

    default:
      break;
  }
}

int_fast8_t radio_parseTriStateSwitch(int_fast16_t value) {
  if (value < 1250) return 0;
  if (value > 1750) return 2;
  return 1;
}

bool radio_parseBiStateSwitch(int_fast16_t value) {
  return value > 1500;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

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
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET);

  HAL_UART_Receive_DMA(&RADIO_RX_UART, radio_rxBuffer, 32);
  HAL_TIM_Base_Start(&PRECISION_TIMER_TIM);
  HAL_TIM_PWM_Start(&MOTORS_PWM_TIM, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&MOTORS_PWM_TIM, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&MOTORS_PWM_TIM, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&MOTORS_PWM_TIM, TIM_CHANNEL_4);
  HAL_ADC_Start_DMA(&BATT_SENSOR_ADC, &batt_adcRawValue, 1);

  HAL_Delay(1000);

  int_fast8_t retry = 0;

  bno055_assignI2C(&IMU_I2C);

  while (!bno055_setup()) {
    if (retry >= I2C_PERIPHERAL_INIT_RETRY_LIMIT) {
      UART_Transmitln_DMA(
          &SERIAL_UART,
          "Failed to initialize BNO055 Inertial Measurement Unit. Entering error state..."
      );
      UART_Transmitln_DMA(
          &WLSER_UART,
          "Failed to initialize BNO055 Inertial Measurement Unit. Entering error state..."
      );
      Error_Handler();
    }
    UART_Transmitln_DMA(&SERIAL_UART, "Failed to initialize BNO055 Inertial Measurement Unit, retrying...");
    UART_Transmitln_DMA(&WLSER_UART, "Failed to initialize BNO055 Inertial Measurement Unit, retrying...");
    HAL_Delay(100);
    retry++;
  }
  bno055_setOperationModeNDOF();

  retry = 0;

  bmp280_init_default_params(&bmp280.params);
  bmp280.params.filter = BMP280_FILTER_16;
  bmp280.params.oversampling_pressure = BMP280_ULTRA_HIGH_RES;
  bmp280.params.oversampling_temperature = BMP280_LOW_POWER;
  bmp280.params.oversampling_humidity = BMP280_SKIPPED;
  bmp280.params.standby = BMP280_STANDBY_05;
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &ALT_I2C;

  while (!bmp280_init(&bmp280, &bmp280.params)) {
    if (retry >= I2C_PERIPHERAL_INIT_RETRY_LIMIT) {
      UART_Transmitln_DMA(&SERIAL_UART, "Failed to initialize BMP280 Altimeter. Entering error state...");
      UART_Transmitln_DMA(&WLSER_UART, "Failed to initialize BMP280 Altimeter. Entering error state...");
      Error_Handler();
    }
    UART_Transmitln_DMA(&SERIAL_UART, "Failed to initialize BMP280 Altimeter, retrying...");
    UART_Transmitln_DMA(&WLSER_UART, "Failed to initialize BMP280 Altimeter, retrying...");
    HAL_Delay(100);
    retry++;
  }

  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    int_fast32_t currentTick_us = GetTickPrecise();
    int_fast32_t deltaTime_us = currentTick_us - pid_lastLoopTime_us;

    bno055_vector_t vector = bno055_getVectorEuler();
    float heading = vector.x;
    float roll = vector.y;
    float pitch = vector.z;

    float temperature, pressure, humidity;  // humidity only for BME280
    bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);

    float targetAngleLimit = radio_targetAngleLimits[radio_parseTriStateSwitch(radio_rxValues[8])];

    float basePower = (float)(radio_rxValues[2] - 1000) / 1000;  // Channel 3 = left joystick vertical

    int_fast16_t rollCommand = radio_rxValues[0] - 1500;  // Channel 1 = right joystick horizontal
    if (rollCommand > -RADIO_DEADZONE && rollCommand < RADIO_DEADZONE) rollCommand = 0;
    float targetRoll = rollCommand * targetAngleLimit / 500.f;

    int_fast16_t pitchCommand = 1500 - radio_rxValues[1];  // Channel 2 = right joystick vertical
    if (pitchCommand > -RADIO_DEADZONE && pitchCommand < RADIO_DEADZONE) pitchCommand = 0;
    float targetPitch = pitchCommand * targetAngleLimit / 500.f;

    int_fast16_t yawCommand = radio_rxValues[3] - 1500;  // Channel 4 = left joystick horizontal
    if (yawCommand > -RADIO_DEADZONE && yawCommand < RADIO_DEADZONE) yawCommand = 0;
    float yaw = yawCommand * targetAngleLimit / 50000.f;

    float delta = deltaTime_us / 1000000.f;
    if (isnanf(delta) || isinff(delta)) { delta = FLT_MIN; }

    float rollError = targetRoll - roll;
    float pitchError = targetPitch - pitch;

    pid_cumulativeRollError += rollError;
    pid_cumulativePitchError += pitchError;

    float cumulativeRollError = pid_cumulativeRollError;
    float cumulativePitchError = pid_cumulativePitchError;

    float deltaRollError = (rollError - pid_lastRollError) / delta;
    float deltaPitchError = (pitchError - pid_lastPitchError) / delta;
    if (isnanf(deltaRollError) || isinff(deltaRollError)) { deltaRollError = 0.f; }
    if (isnanf(deltaPitchError) || isinff(deltaPitchError)) { deltaPitchError = 0.f; }

    float proportionalRollCorrection = rollError * PID_PROPORTIONAL_GAIN / 100.f;
    float proportionalPitchCorrection = pitchError * PID_PROPORTIONAL_GAIN / 100.f;
    float integralRollCorrection = cumulativeRollError * PID_INTEGRAL_GAIN / 100.f;
    float integralPitchCorrection = cumulativePitchError * PID_INTEGRAL_GAIN / 100.f;
    float derivativeRollCorrection = deltaRollError * PID_DERIVATIVE_GAIN / 100.f;
    float derivativePitchCorrection = deltaPitchError * PID_DERIVATIVE_GAIN / 100.f;

    float rollCorrection = proportionalRollCorrection + integralRollCorrection + derivativeRollCorrection;
    float pitchCorrection = proportionalPitchCorrection + integralPitchCorrection + derivativePitchCorrection;

    float cosineLossCorrection = 1.f / (cosf(roll / 180.f * M_PI) * cosf(pitch / 180.f * M_PI));
    if (isnanf(cosineLossCorrection) || isinff(cosineLossCorrection)) { cosineLossCorrection = 1.f; }

    float motor1Power_FL = basePower * cosineLossCorrection + rollCorrection - pitchCorrection + yaw;
    float motor2Power_RR = basePower * cosineLossCorrection - rollCorrection + pitchCorrection + yaw;
    float motor3Power_FR = basePower * cosineLossCorrection - rollCorrection - pitchCorrection - yaw;
    float motor4Power_RL = basePower * cosineLossCorrection + rollCorrection + pitchCorrection - yaw;

    pid_lastRollError = rollError;
    pid_lastPitchError = pitchError;

    motors_SetPower(MOTOR_1_FL, motor1Power_FL);
    motors_SetPower(MOTOR_2_RR, motor2Power_RR);
    motors_SetPower(MOTOR_3_FR, motor3Power_FR);
    motors_SetPower(MOTOR_4_RL, motor4Power_RL);

    pid_lastLoopTime_us = currentTick_us;
    telemetry_iterCount++;

    if (currentTick_us - telemetry_lastTransmission_us > TELEMETRY_INTERVAL) {
      char buffer[2048];
      sprintf(
          buffer,
          "--------------------------------------------------------------------------------\r\n"
          "Radio       : %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d\r\n"
          "Target      : %4.2fY %4.2fR %4.2fP\r\n"
          "Orientation : %4.2fH %4.2fR %4.2fP\r\n"
          "Errors      : %2.4fR %2.4fP\r\n"
          "Corrections : %1.5fY %1.5fR %1.5fP %1.5fCos \r\n"
          "Atmos       : %.2fC, %.2fPa\r\n"
          "Powers      : %3.1f%% %3.1f%%\r\n"
          "              %3.1f%% %3.1f%%\r\n"
          "Batt        : %.2f V\r\n"
          "It/s        : %d\r\n",
          radio_rxValues[0],
          radio_rxValues[1],
          radio_rxValues[2],
          radio_rxValues[3],
          radio_rxValues[4],
          radio_rxValues[5],
          radio_rxValues[6],
          radio_rxValues[7],
          radio_rxValues[8],
          radio_rxValues[9],
          yaw,
          targetRoll,
          targetPitch,
          heading,
          roll,
          pitch,
          rollError,
          pitchError,
          yaw,
          rollCorrection,
          pitchCorrection,
          cosineLossCorrection,
          temperature,
          pressure,
          motor1Power_FL * 100,
          motor3Power_FR * 100,
          motor4Power_RL * 100,
          motor2Power_RR * 100,
          batt_adcRawValue * BATT_VOLTAGE_SCALE / 0xFFF,
          telemetry_iterCount * 1000 / TELEMETRY_INTERVAL
      );

      size_t len = strlen(buffer);
      HAL_UART_Transmit_DMA(&SERIAL_UART, (uint8_t *)buffer, len);
      HAL_UART_Transmit_DMA(&WLSER_UART, (uint8_t *)buffer, len);

      telemetry_lastTransmission_us = currentTick_us;
      telemetry_iterCount = 0;
    }

    uint8_t temp;
    if (HAL_UART_Receive(&SERIAL_UART, &temp, 1, 0) == HAL_OK || HAL_UART_Receive(&WLSER_UART, &temp, 1, 0) == HAL_OK) {
      motors_SetPower(MOTOR_1_FL, 0.f);
      motors_SetPower(MOTOR_2_RR, 0.f);
      motors_SetPower(MOTOR_3_FR, 0.f);
      motors_SetPower(MOTOR_4_RL, 0.f);

      char msg[] = "E-STOP Activated, shutting down...\r\n";
      size_t len = strlen(msg);

      while (HAL_UART_Transmit(&SERIAL_UART, (uint8_t *)msg, len, 250) == HAL_BUSY);
      while (HAL_UART_Transmit(&WLSER_UART, (uint8_t *)msg, len, 250) == HAL_BUSY);

      Error_Handler();
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) { Error_Handler(); }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) { Error_Handler(); }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {
  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETH_Init(void) {
  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK) { Error_Handler(); }

  memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {
  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief I2C4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C4_Init(void) {
  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x20404768;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK) { Error_Handler(); }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 108 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 108 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2500 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {
  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 1500000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {
  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {
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
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
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
  HAL_GPIO_WritePin(GPIOB, LED_Green_Pin | LED_Red_Pin | LED_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_B1_Pin */
  GPIO_InitStruct.Pin = BTN_B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Green_Pin LED_Red_Pin LED_Blue_Pin */
  GPIO_InitStruct.Pin = LED_Green_Pin | LED_Red_Pin | LED_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &PRECISION_TIMER_TIM) { precisionTimer_acc_us += 1000000; }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &RADIO_RX_UART) {
    uint16_t checksum = 0;
    for (int i = 0; i < 30; i++) { checksum += radio_rxBuffer[i]; }
    checksum += ((uint16_t)(radio_rxBuffer[31]) << 8) | radio_rxBuffer[30];
    if (checksum == 0xFFFF && radio_rxBuffer[0] == 0x20 && radio_rxBuffer[1] == 0x40) {
      for (int channel = 0; channel < 10; channel++) {
        uint16_t value = ((uint16_t)(radio_rxBuffer[channel * 2 + 3]) << 8) | radio_rxBuffer[channel * 2 + 2];
        if (value < 1000) value = 1000;
        if (value > 2000) value = 2000;
        radio_rxValues[channel] = value;
      }
      radio_consecutiveErrors = 0;
      // UART_Transmitln_DMA(&SERIAL_UART, "Received and processed valid radio packet");
      HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, HAL_GetTick() & (1 << 6) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
      radio_consecutiveErrors++;
      // UART_Transmitf_DMA(
      //     &SERIAL_UART,
      //     "Received invalid radio packet. Checksum: 0x%04X, Length: 0x%02X, Protocol: 0x%02X\r\n",
      //     checksum,
      //     radio_rxBuffer[0],
      //     radio_rxBuffer[1]
      // );
      HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, HAL_GetTick() & (1 << 6) ? GPIO_PIN_SET : GPIO_PIN_RESET);

      if (radio_consecutiveErrors >= RADIO_CONSECUTIVE_ERROR_THRESHOLD) {
        UART_Transmitf_DMA(
            &SERIAL_UART,
            "Received %d consecutive invalid radio packets, skipping next byte.\r\n",
            radio_consecutiveErrors
        );
        uint8_t temp;
        HAL_UART_Receive(&RADIO_RX_UART, &temp, 1, 20);
      }
    }
    HAL_UART_Receive_DMA(&RADIO_RX_UART, radio_rxBuffer, 32);
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  motors_SetPower(MOTOR_1_FL, 0.f);
  motors_SetPower(MOTOR_2_RR, 0.f);
  motors_SetPower(MOTOR_3_FR, 0.f);
  motors_SetPower(MOTOR_4_RL, 0.f);

  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);
  while (1) {
    HAL_GPIO_TogglePin(LED_Red_GPIO_Port, LED_Red_Pin);
    HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
    HAL_GPIO_TogglePin(LED_Blue_GPIO_Port, LED_Blue_Pin);
    for (uint_fast32_t i = 0; i < 1000000; i++) {}
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
