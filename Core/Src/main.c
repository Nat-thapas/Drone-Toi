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
#include "inttypes.h"
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

#define WAIT_FOR_CALIBRATION  // Comment to not wait for IMU calibration on startup

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

#define I2C_PERIPHERAL_INIT_RETRY_LIMIT 10

#define MOTOR_1_FL_PWM_CCR htim3.Instance->CCR4
#define MOTOR_2_RR_PWM_CCR htim3.Instance->CCR1
#define MOTOR_3_FR_PWM_CCR htim3.Instance->CCR3
#define MOTOR_4_RL_PWM_CCR htim3.Instance->CCR2

#define BATT_SENSOR_ADC hadc1
#define BATT_VOLTAGE_MULTIPLIER 19.375f

#define EXTI_DEBOUNCE 250

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
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

static bool override_outputDisabled = false;
static int_fast32_t interrupts_lastTriggers[16] = {};

static uint32_t batt_adcRawValue = 0;
static int_fast16_t batt_mavgValue = -1;

static int_fast32_t telemetry_interval = 100000;  // us.
static int_fast32_t telemetry_lastTransmission_us = 0;
static int_fast32_t telemetry_mavgProcTime = -1;
static int_fast16_t telemetry_iterationCount = 0;

// static const char text_status[] = "\x1B[31mBAD\x1B[0m\0\x1B[32mOK \x1B[0m";

static uint8_t radio_rxBuffer[32] = {};
static int_fast16_t radio_rxValues[10] = {
    1500,
    1500,
    1000,
    1500,
    1000,
    1000,
    1000,
    1000,
    1000,
    1000,
};
// Channel 1, 2, 4: Value that is within +- deadzone of middle (1500) will be set to middle (1500)
// Channel 3: Value that is within +- deadzone of min (1000) or max (2000) will be set to min or max
static int_fast16_t radio_deadZone = 25;
static int_fast16_t radio_consecutiveErrors = 0;
static const float radio_targetAngleLimits[3] = {5.f, 10.f, 30.f};

static float control_yawAngleRate = 3.f;  // 3 * angleLimit degrees / sec at max input

static float imu_trimHeading = 0.f;
static float imu_trimPitch = 4.5f;
static float imu_trimRoll = -0.5f;
// Lower value = lower frequency cutoff, the filtering algorithm is exponetially weighted moving average
static float imu_yawRateFilteringAlpha = 0.125f;
static float imu_pitchRateFilteringAlpha = 0.25f;
static float imu_rollRateFilteringAlpha = 0.25f;

static int_fast32_t pid_minLoopPeriod = 10000;  // us.
static int_fast32_t pid_lastLoopTime_us = 0;

static float pid_y_proportionalGain = 0.25f;
static float pid_y_integralGain = 0.3f;
static float pid_y_derivativeGain = 0.2f;
static float pid_y_cumulativeErrorLimit = 1.5f;

static float pid_pr_proportionalGain = 0.15f;
static float pid_pr_integralGain = 0.2f;
static float pid_pr_derivativeGain = 0.1f;
static float pid_pr_cumulativeErrorLimit = 1.5f;

// Integrator will only be active if current error is less than or equal to threshold degrees
static float pid_y_integratorActiveThreshold = 5.f;
static float pid_pr_integratorActiveThreshold = 5.f;
static float pid_mavgYawRate = NAN;
static float pid_mavgPitchRate = NAN;
static float pid_mavgRollRate = NAN;
static float pid_cumulativeYawError = 0.f;
static float pid_cumulativeRollError = 0.f;
static float pid_cumulativePitchError = 0.f;
static float pid_targetHeading = NAN;

static uint8_t serial_rxBuffer[32] = {};
static uint8_t wlser_rxBuffer[32] = {};

static char command_buffer[256] = {};
static size_t command_bufferIndex = 0;
static bool command_ready = false;

static BMP280_HandleTypedef bmp280;

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

static inline int_fast32_t GetTickPrecise() {
  return PRECISION_TIMER_COUNT;
}

void Serial_Transmit_DMA(char *str) {
  HAL_UART_Transmit_DMA(&SERIAL_UART, (uint8_t *)str, strlen(str));
  HAL_UART_Transmit_DMA(&WLSER_UART, (uint8_t *)str, strlen(str));
}

void Serial_Transmitf_DMA(const char *format, ...) {
  char buffer[2048];
  va_list args;

  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  Serial_Transmit_DMA(buffer);
}

void Serial_Transmitln_DMA(char *str) {
  Serial_Transmitf_DMA("%s\r\n", str);
}

static inline int_fast8_t radio_parseTriStateSwitch(int_fast16_t value) {
  return value < 1250 ? 0 : (value > 1750 ? 2 : 1);
}

static inline bool radio_parseBiStateSwitch(int_fast16_t value) {
  return value > 1500;
}

static inline bool strprei(const char *str, const char *pre) {
  return strncmpi(str, pre, strlen(pre)) == 0;
}

static inline float clamp(float val, float min, float max) {
  return val < min ? min : (val > max ? max : val);
}

static inline float cmodf(float a, float b) {
  return fmod((fmod(a, b) + b), b);
}

static void DelayPrecise(uint32_t delay) {
  uint32_t start = PRECISION_TIMER_COUNT;
  while (PRECISION_TIMER_COUNT - start < delay);
}

static void command_handle() {
  char *command = command_buffer;
  size_t len = command_bufferIndex;

  command_ready = false;
  command_bufferIndex = 0;

  if (len >= 4 && strprei(command, "stop")) {
    // stop
    command += 4;
    len -= 4;
    while (*command == 0x20) {
      command++;
      len--;
    }

    MOTOR_1_FL_PWM_CCR = 4000;
    MOTOR_2_RR_PWM_CCR = 4000;
    MOTOR_3_FR_PWM_CCR = 4000;
    MOTOR_4_RL_PWM_CCR = 4000;

    char msg[] = "\r\nE-STOP Activated, shutting down...\r\n";
    size_t len = strlen(msg);

    while (HAL_UART_Transmit(&SERIAL_UART, (uint8_t *)msg, len, 250) == HAL_BUSY);
    while (HAL_UART_Transmit(&WLSER_UART, (uint8_t *)msg, len, 250) == HAL_BUSY);

    Error_Handler();
    return;
  } else if (len >= 3 && strprei(command, "set")) {
    // set
    command += 3;
    len -= 3;
    while (*command == 0x20) {
      command++;
      len--;
    }

    if (len >= 3 && strprei(command, "pid")) {
      // set pid
      command += 3;
      len -= 3;
      while (*command == 0x20) {
        command++;
        len--;
      }

      if (len >= 1 && strprei(command, "y")) {
        // set pid y
        command += 1;
        len -= 1;
        while (*command == 0x20) {
          command++;
          len--;
        }

        if (len >= 1 && strprei(command, "p")) {
          // set pid y p
          command += 1;
          len -= 1;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &pid_y_proportionalGain) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        } else if (len >= 1 && strprei(command, "i")) {
          // set pid y i
          command += 1;
          len -= 1;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &pid_y_integralGain) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        } else if (len >= 1 && strprei(command, "d")) {
          // set pid y d
          command += 1;
          len -= 1;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &pid_y_derivativeGain) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        } else if (len >= 3 && strprei(command, "cel")) {
          // set pid y cel
          command += 3;
          len -= 3;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &pid_y_cumulativeErrorLimit) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        } else if (len >= 3 && strprei(command, "iat")) {
          // set pid y iat
          command += 3;
          len -= 3;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &pid_y_integratorActiveThreshold) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        }
      } else if (len >= 2 && strprei(command, "pr")) {
        // set pid pr
        command += 2;
        len -= 2;
        while (*command == 0x20) {
          command++;
          len--;
        }

        if (len >= 1 && strprei(command, "p")) {
          // set pid pr p
          command += 1;
          len -= 1;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &pid_pr_proportionalGain) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        } else if (len >= 1 && strprei(command, "i")) {
          // set pid pr i
          command += 1;
          len -= 1;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &pid_pr_integralGain) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        } else if (len >= 1 && strprei(command, "d")) {
          // set pid pr d
          command += 1;
          len -= 1;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &pid_pr_derivativeGain) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        } else if (len >= 3 && strprei(command, "cel")) {
          // set pid cel
          command += 3;
          len -= 3;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &pid_pr_cumulativeErrorLimit) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        } else if (len >= 3 && strprei(command, "iat")) {
          // set pid iat
          command += 3;
          len -= 3;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &pid_pr_integratorActiveThreshold) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        }
      } else if (len >= 3 && strprei(command, "mli")) {
        // set pid mli
        command += 3;
        len -= 3;
        while (*command == 0x20) {
          command++;
          len--;
        }

        if (sscanf(command, "%" PRIdFAST32, &pid_minLoopPeriod) == 1) {
          strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
        } else {
          strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
        }
        return;
      }
    } else if (len >= 4 && strprei(command, "trim")) {
      // set trim
      command += 4;
      len -= 4;
      while (*command == 0x20) {
        command++;
        len--;
      }

      if (len >= 1 && strprei(command, "h")) {
        // set trim h
        command += 1;
        len -= 1;
        while (*command == 0x20) {
          command++;
          len--;
        }

        if (sscanf(command, "%f", &imu_trimHeading) == 1) {
          strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
        } else {
          strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
        }
        return;
      } else if (len >= 1 && strprei(command, "p")) {
        // set trim p
        command += 1;
        len -= 1;
        while (*command == 0x20) {
          command++;
          len--;
        }

        if (sscanf(command, "%f", &imu_trimPitch) == 1) {
          strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
        } else {
          strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
        }
        return;
      } else if (len >= 1 && strprei(command, "r")) {
        // set trim r
        command += 1;
        len -= 1;
        while (*command == 0x20) {
          command++;
          len--;
        }

        if (sscanf(command, "%f", &imu_trimRoll) == 1) {
          strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
        } else {
          strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
        }
        return;
      }
    } else if (len >= 5 && strprei(command, "radio")) {
      // set radio
      command += 5;
      len -= 5;
      while (*command == 0x20) {
        command++;
        len--;
      }

      if (len >= 8 && strprei(command, "deadzone")) {
        // set radio deadzone
        command += 8;
        len -= 8;
        while (*command == 0x20) {
          command++;
          len--;
        }

        if (sscanf(command, "%" PRIdFAST16, &radio_deadZone) == 1) {
          strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
        } else {
          strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
        }
        return;
      }
    } else if (len >= 7 && strprei(command, "control")) {
      // set control
      command += 7;
      len -= 7;
      while (*command == 0x20) {
        command++;
        len--;
      }

      if (len >= 3 && strprei(command, "yar")) {
        // set radio yar
        command += 3;
        len -= 3;
        while (*command == 0x20) {
          command++;
          len--;
        }

        if (sscanf(command, "%f", &control_yawAngleRate) == 1) {
          strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
        } else {
          strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
        }
        return;
      }
    } else if (len >= 9 && strprei(command, "telemetry")) {
      // set telemetry
      command += 9;
      len -= 9;
      while (*command == 0x20) {
        command++;
        len--;
      }

      if (len >= 8 && strprei(command, "interval")) {
        // set telemetry interval
        command += 8;
        len -= 8;
        while (*command == 0x20) {
          command++;
          len--;
        }

        if (sscanf(command, "%" PRIdFAST32, &telemetry_interval) == 1) {
          strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
        } else {
          strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
        }
        return;
      }
    } else if (len >= 3 && strprei(command, "imu")) {
      // set imu
      command += 3;
      len -= 3;
      while (*command == 0x20) {
        command++;
        len--;
      }

      if (len >= 4 && strprei(command, "lpfa")) {
        // set imu lpfa
        command += 4;
        len -= 4;
        while (*command == 0x20) {
          command++;
          len--;
        }

        if (len >= 2 && strprei(command, "yr")) {
          // set imu lpfa yr
          command += 2;
          len -= 2;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &imu_yawRateFilteringAlpha) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        } else if (len >= 2 && strprei(command, "pr")) {
          // set imu lpfa pr
          command += 2;
          len -= 2;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &imu_pitchRateFilteringAlpha) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        } else if (len >= 2 && strprei(command, "rr")) {
          // set imu lpfa rr
          command += 2;
          len -= 2;
          while (*command == 0x20) {
            command++;
            len--;
          }

          if (sscanf(command, "%f", &imu_rollRateFilteringAlpha) == 1) {
            strcpy(command_buffer, "\x1B[32mOK\x1B[0m");
          } else {
            strcpy(command_buffer, "\x1B[31mInvalid\x1B[0m");
          }
          return;
        }
      }
    }
  }

  strcpy(command_buffer, "\x1B[33mUnknown\x1B[0m");
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
  HAL_UARTEx_ReceiveToIdle_DMA(&SERIAL_UART, serial_rxBuffer, 32);
  HAL_UARTEx_ReceiveToIdle_DMA(&WLSER_UART, wlser_rxBuffer, 32);
  HAL_TIM_Base_Start_IT(&PRECISION_TIMER_TIM);
  HAL_TIM_PWM_Start(&MOTORS_PWM_TIM, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&MOTORS_PWM_TIM, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&MOTORS_PWM_TIM, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&MOTORS_PWM_TIM, TIM_CHANNEL_4);
  HAL_ADC_Start_DMA(&BATT_SENSOR_ADC, &batt_adcRawValue, 1);

  Serial_Transmit_DMA(
      "##################################################\r\n"
      "#                                                #\r\n"
      "#                                                #\r\n"
      "#   ____                           _____     _   #\r\n"
      "#  |  _ \\ _ __ ___  _ __   ___    |_   _|__ (_)  #\r\n"
      "#  | | | | '__/ _ \\| '_ \\ / _ \\_____| |/ _ \\| |  #\r\n"
      "#  | |_| | | | (_) | | | |  __/_____| | (_) | |  #\r\n"
      "#  |____/|_|  \\___/|_| |_|\\___|     |_|\\___/|_|  #\r\n"
      "#                                                #\r\n"
      "#                                                #\r\n"
      "#                    Welcome                     #\r\n"
      "#                                                #\r\n"
      "#             Initializing systems...            #\r\n"
      "#                                                #\r\n"
      "#                                                #\r\n"
      "##################################################\r\n"
  );
  HAL_Delay(250);

  Serial_Transmitln_DMA("Initializing BNO055 Inertial Measurement Unit...");
  HAL_Delay(100);

  int_fast8_t retry = 0;

  bno055_assignI2C(&IMU_I2C);

  while (!bno055_setup()) {
    if (retry >= I2C_PERIPHERAL_INIT_RETRY_LIMIT) {
      Serial_Transmitln_DMA("Failed to initialize BNO055 Inertial Measurement Unit. Entering error state...");
      Error_Handler();
    }
    Serial_Transmitln_DMA("Failed to initialize BNO055 Inertial Measurement Unit, retrying...");
    HAL_Delay(100);
    retry++;
  }
  retry = 0;
  while (!bno055_setOperationModeNDOF()) {
    if (retry >= I2C_PERIPHERAL_INIT_RETRY_LIMIT) {
      Serial_Transmitln_DMA("Failed to configure BNO055 Inertial Measurement Unit. Entering error state...");
      Error_Handler();
    }
    Serial_Transmitln_DMA("Failed to configure BNO055 Inertial Measurement Unit, retrying...");
    HAL_Delay(100);
    retry++;
  }

#ifdef WAIT_FOR_CALIBRATION
  bool calibrationStateValid;
  bno055_calibration_state_t calibrationState;
  int_fast8_t i = 0;
  Serial_Transmitln_DMA("Waiting for IMU calibration...\r\n");
  do {
    calibrationStateValid = bno055_getCalibrationState(&calibrationState);
    Serial_Transmitf_DMA(
        "\x1b[1F%c > System: %d, Gyroscope: %d, Accelerometer: %d, Magnetometer: %d\r\n",
        "|/-\\|/-\\"[i % 8],
        calibrationState.sys,
        calibrationState.gyro,
        calibrationState.accel,
        calibrationState.mag
    );
    i++;
    HAL_GPIO_TogglePin(LED_Blue_GPIO_Port, LED_Blue_Pin);
    HAL_Delay(500);
  } while (!calibrationStateValid || calibrationState.sys != 0x03 || calibrationState.gyro != 0x03 ||
           calibrationState.mag != 0x03);

  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET);
#endif  // WAIT_FOR_CALIBRATION

  Serial_Transmitln_DMA("BNO055 Inertial Measurement Unit initialization complete");
  HAL_Delay(100);

  Serial_Transmitln_DMA("Initializing BMP280 Barometer...");
  HAL_Delay(100);

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
      Serial_Transmitln_DMA("Failed to initialize BMP280 Barometer. Entering error state...");
      Error_Handler();
    }
    Serial_Transmitln_DMA("Failed to initialize BMP280 Barometer, retrying...");
    HAL_Delay(100);
    retry++;
  }

  Serial_Transmitln_DMA("BMP280 Barometer initialization complete");
  HAL_Delay(100);

  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);

  Serial_Transmit_DMA("\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, HAL_GetTick() & (1 << 6) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    int_fast32_t currentTick_us = GetTickPrecise();
    int_fast32_t deltaTime_us = currentTick_us - pid_lastLoopTime_us;

    while (deltaTime_us < pid_minLoopPeriod) {
      currentTick_us = GetTickPrecise();
      deltaTime_us = currentTick_us - pid_lastLoopTime_us;
    }

    float delta = deltaTime_us / 1000000.f;

    if (batt_mavgValue == -1) {
      batt_mavgValue = batt_adcRawValue;
    } else {
      batt_mavgValue = (batt_mavgValue * 15 / 16) + (batt_adcRawValue / 16);
    }

    // int_fast32_t imuAcquisitionStart = GetTickPrecise();
    bno055_vector_t orientationVector;
    bno055_vector_t angularVelocityVector;
    bool orientationDataValid =
        bno055_getVectorGyroscope(&angularVelocityVector) && bno055_getVectorEuler(&orientationVector);
    // int_fast32_t imuAcquisitionTime = GetTickPrecise() - imuAcquisitionStart;

    // int_fast32_t altAcquisitionStart = GetTickPrecise();
    float temperature, pressure, humidity;  // humidity only for BME280
    /* bool atmosDataValid = */ bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
    // int_fast32_t altAcquisitionTime = GetTickPrecise() - altAcquisitionStart;

    // 1st switch, up = false = output off, down = true = output on
    bool outputEnabled = radio_parseBiStateSwitch(radio_rxValues[6]);
    // 2nd switch, up = false = FA on, down = true = FA off
    bool faDisabled = radio_parseBiStateSwitch(radio_rxValues[7]);
    // 3rd switch (tri-state)
    float targetAngleLimit = radio_targetAngleLimits[radio_parseTriStateSwitch(radio_rxValues[8])];
    // 4th switch, up = false = integrator off, down = true = integrator on
    bool integratorEnabled = radio_parseBiStateSwitch(radio_rxValues[9]);

    // float pitchTrim = (radio_rxValues[4] - 1500.f) / 100.f;
    // float rollTrim = (radio_rxValues[5] - 1500.f) / 100.f;
    float headingTrim = imu_trimHeading;
    float pitchTrim = imu_trimPitch;
    float rollTrim = imu_trimRoll;

    float heading = cmodf(orientationVector.x - headingTrim, 360.f);
    float pitch = orientationVector.y - pitchTrim;  // + = nose up
    float roll = -orientationVector.z - rollTrim;   // + = right wing down

    float yawRate = -angularVelocityVector.z;
    float pitchRate = -angularVelocityVector.y;
    float rollRate = angularVelocityVector.x;

    if (isnanf(pid_mavgYawRate) || isinff(pid_mavgYawRate)) {
      pid_mavgYawRate = yawRate;
    } else {
      pid_mavgYawRate = (pid_mavgYawRate * (1.f - imu_yawRateFilteringAlpha)) + (yawRate * imu_yawRateFilteringAlpha);
    }
    if (isnanf(pid_mavgPitchRate) || isinff(pid_mavgPitchRate)) {
      pid_mavgPitchRate = pitchRate;
    } else {
      pid_mavgPitchRate =
          (pid_mavgPitchRate * (1.f - imu_pitchRateFilteringAlpha)) + (pitchRate * imu_pitchRateFilteringAlpha);
    }
    if (isnanf(pid_mavgRollRate) || isinff(pid_mavgRollRate)) {
      pid_mavgRollRate = rollRate;
    } else {
      pid_mavgRollRate =
          (pid_mavgRollRate * (1.f - imu_rollRateFilteringAlpha)) + (rollRate * imu_rollRateFilteringAlpha);
    }

    yawRate = pid_mavgYawRate;
    pitchRate = pid_mavgPitchRate;
    rollRate = pid_mavgRollRate;

    float altitude = 44330.f * (1 - powf(pressure / 101325.f, 1 / 5.255f));

    // float pr_proportionalGain = (radio_rxValues[4] - 1000.f) / 1000.f;
    // float pr_derivativeGain = (radio_rxValues[5] - 1000.f) / 1000.f;
    float y_proportionalGain = pid_y_proportionalGain;
    float y_integralGain = pid_y_integralGain;
    float y_derivativeGain = pid_y_derivativeGain;
    float pr_proportionalGain = pid_pr_proportionalGain;
    float pr_integralGain = pid_pr_integralGain;
    float pr_derivativeGain = pid_pr_derivativeGain;

    float targetHeading = pid_targetHeading, targetPitch = 0.f, targetRoll = 0.f;
    float yawError = 0.f, pitchError = 0.f, rollError = 0.f;
    float proportionalYawCorrection = 0.f, integralYawCorrection = 0.f, derivativeYawCorrection = 0.f;
    float proportionalPitchCorrection = 0.f, integralPitchCorrection = 0.f, derivativePitchCorrection = 0.f;
    float proportionalRollCorrection = 0.f, integralRollCorrection = 0.f, derivativeRollCorrection = 0.f;
    float yawCorrection = 0.f, pitchCorrection = 0.f, rollCorrection = 0.f, cosineLossCorrection = 1.f;
    float motor1Power_FL = 0.f, motor2Power_RR = 0.f, motor3Power_FR = 0.f, motor4Power_RL = 0.f;

    int_fast16_t altCommand = radio_rxValues[2] - 1000;  // Channel 3 = left joystick vertical
    altCommand = altCommand <= radio_deadZone ? 0 : (altCommand >= 1000 - radio_deadZone) ? 1000 : altCommand;

    int_fast16_t yawCommand = radio_rxValues[3] - 1500;  // Channel 4 = left joystick horizontal
    yawCommand = yawCommand >= -radio_deadZone && yawCommand <= radio_deadZone ? 0 : yawCommand;

    int_fast16_t pitchCommand = 1500 - radio_rxValues[1];  // Channel 2 = right joystick vertical
    pitchCommand = pitchCommand >= -radio_deadZone && pitchCommand <= radio_deadZone ? 0 : pitchCommand;

    int_fast16_t rollCommand = radio_rxValues[0] - 1500;  // Channel 1 = right joystick horizontal
    rollCommand = rollCommand >= -radio_deadZone && rollCommand <= radio_deadZone ? 0 : rollCommand;

    if (!integratorEnabled) {
      pid_cumulativeYawError = 0.f;
      pid_cumulativePitchError = 0.f;
      pid_cumulativeRollError = 0.f;
    }

    if (faDisabled) {
      pid_targetHeading = NAN;

      pid_cumulativeYawError = 0.f;
      pid_cumulativePitchError = 0.f;
      pid_cumulativeRollError = 0.f;

      float basePower = altCommand / 1000.f;

      yawCorrection = yawCommand * targetAngleLimit / 600000.f;
      pitchCorrection = pitchCommand * targetAngleLimit / 600000.f;
      rollCorrection = rollCommand * targetAngleLimit / 600000.f;

      motor1Power_FL = basePower - yawCorrection + pitchCorrection + rollCorrection;
      motor2Power_RR = basePower - yawCorrection - pitchCorrection - rollCorrection;
      motor3Power_FR = basePower + yawCorrection + pitchCorrection - rollCorrection;
      motor4Power_RL = basePower + yawCorrection - pitchCorrection + rollCorrection;
    } else if (!orientationDataValid) {
      pid_targetHeading = NAN;

      float basePower = altCommand / 1000.f;

      motor1Power_FL = basePower;
      motor2Power_RR = basePower;
      motor3Power_FR = basePower;
      motor4Power_RL = basePower;
    } else if (altCommand == 0) {
      pid_targetHeading = NAN;

      motor1Power_FL = 0.f;
      motor2Power_RR = 0.f;
      motor3Power_FR = 0.f;
      motor4Power_RL = 0.f;
    } else if (altCommand == 1000) {
      motor1Power_FL = 1.f;
      motor2Power_RR = 1.f;
      motor3Power_FR = 1.f;
      motor4Power_RL = 1.f;
    } else {
      float basePower = altCommand / 1000.f;

      if (isnanf(targetHeading) || isinff(targetHeading)) { targetHeading = heading; }
      targetHeading =
          cmodf(targetHeading + (yawCommand / 500.f * targetAngleLimit * control_yawAngleRate * delta), 360.f);
      targetPitch = pitchCommand * targetAngleLimit / 500.f;
      targetRoll = rollCommand * targetAngleLimit / 500.f;

      pid_targetHeading = targetHeading;

      yawError = cmodf(targetHeading - heading + 180.f, 360.f) - 180.f;
      pitchError = targetPitch - pitch;
      rollError = targetRoll - roll;

      float cumulativeYawError = pid_cumulativeYawError;
      float cumulativePitchError = pid_cumulativePitchError;
      float cumulativeRollError = pid_cumulativeRollError;

      proportionalYawCorrection = yawError * y_proportionalGain / 100.f;
      integralYawCorrection = cumulativeYawError * y_integralGain / 100.f;
      derivativeYawCorrection = -yawRate * y_derivativeGain / 100.f;

      proportionalPitchCorrection = pitchError * pr_proportionalGain / 100.f;
      integralPitchCorrection = cumulativePitchError * pr_integralGain / 100.f;
      derivativePitchCorrection = -pitchRate * pr_derivativeGain / 100.f;

      proportionalRollCorrection = rollError * pr_proportionalGain / 100.f;
      integralRollCorrection = cumulativeRollError * pr_integralGain / 100.f;
      derivativeRollCorrection = -rollRate * pr_derivativeGain / 100.f;

      yawCorrection = proportionalYawCorrection + integralYawCorrection + derivativeYawCorrection;
      pitchCorrection = proportionalPitchCorrection + integralPitchCorrection + derivativePitchCorrection;
      rollCorrection = proportionalRollCorrection + integralRollCorrection + derivativeRollCorrection;

      cosineLossCorrection = clamp(1.f / (cosf(roll / 180.f * M_PI) * cosf(pitch / 180.f * M_PI)), 1.f, 1.25f);
      if (isnanf(cosineLossCorrection) || isinff(cosineLossCorrection)) { cosineLossCorrection = 1.f; }
      // cosineLossCorrection = 1.f;  // Uncomment to disable cosine loss correction

      motor1Power_FL = basePower * cosineLossCorrection - yawCorrection + pitchCorrection + rollCorrection;
      motor2Power_RR = basePower * cosineLossCorrection - yawCorrection - pitchCorrection - rollCorrection;
      motor3Power_FR = basePower * cosineLossCorrection + yawCorrection + pitchCorrection - rollCorrection;
      motor4Power_RL = basePower * cosineLossCorrection + yawCorrection - pitchCorrection + rollCorrection;

      bool controlSaturated =
          (motor1Power_FL < 0.f || motor1Power_FL > 1.f) || (motor2Power_RR < 0.f || motor2Power_RR > 1.f) ||
          (motor3Power_FR < 0.f || motor3Power_FR > 1.f) || (motor4Power_RL < 0.f || motor4Power_RL > 1.f);

      if (integratorEnabled && !controlSaturated && yawError >= -pid_y_integratorActiveThreshold &&
          yawError <= pid_y_integratorActiveThreshold) {
        pid_cumulativeYawError += yawError * delta;
        pid_cumulativeYawError = clamp(pid_cumulativeYawError, -pid_y_cumulativeErrorLimit, pid_y_cumulativeErrorLimit);
      }
      if (integratorEnabled && !controlSaturated && pitchError >= -pid_pr_integratorActiveThreshold &&
          pitchError <= pid_pr_integratorActiveThreshold) {
        pid_cumulativePitchError += pitchError * delta;
        pid_cumulativePitchError =
            clamp(pid_cumulativePitchError, -pid_pr_cumulativeErrorLimit, pid_pr_cumulativeErrorLimit);
      }
      if (integratorEnabled && !controlSaturated && rollError >= -pid_pr_integratorActiveThreshold &&
          rollError <= pid_pr_integratorActiveThreshold) {
        pid_cumulativeRollError += rollError * delta;
        pid_cumulativeRollError =
            clamp(pid_cumulativeRollError, -pid_pr_cumulativeErrorLimit, pid_pr_cumulativeErrorLimit);
      }
    }

    int_fast16_t motor1Pwm_FL = clamp(motor1Power_FL, 0.f, 1.f) * 4000.f + 4000.f;
    int_fast16_t motor2Pwm_RR = clamp(motor2Power_RR, 0.f, 1.f) * 4000.f + 4000.f;
    int_fast16_t motor3Pwm_FR = clamp(motor3Power_FR, 0.f, 1.f) * 4000.f + 4000.f;
    int_fast16_t motor4Pwm_RL = clamp(motor4Power_RL, 0.f, 1.f) * 4000.f + 4000.f;

    motor1Pwm_FL = motor1Pwm_FL < 4000 ? 4000 : (motor1Pwm_FL > 8000 ? 8000 : motor1Pwm_FL);
    motor2Pwm_RR = motor2Pwm_RR < 4000 ? 4000 : (motor2Pwm_RR > 8000 ? 8000 : motor2Pwm_RR);
    motor3Pwm_FR = motor3Pwm_FR < 4000 ? 4000 : (motor3Pwm_FR > 8000 ? 8000 : motor3Pwm_FR);
    motor4Pwm_RL = motor4Pwm_RL < 4000 ? 4000 : (motor4Pwm_RL > 8000 ? 8000 : motor4Pwm_RL);

    MOTOR_1_FL_PWM_CCR = (outputEnabled && !override_outputDisabled) ? motor1Pwm_FL : 4000;
    MOTOR_2_RR_PWM_CCR = (outputEnabled && !override_outputDisabled) ? motor2Pwm_RR : 4000;
    MOTOR_3_FR_PWM_CCR = (outputEnabled && !override_outputDisabled) ? motor3Pwm_FR : 4000;
    MOTOR_4_RL_PWM_CCR = (outputEnabled && !override_outputDisabled) ? motor4Pwm_RL : 4000;

    if (currentTick_us - telemetry_lastTransmission_us > telemetry_interval) {
      Serial_Transmitf_DMA(
          "\x1B[?25l"  // Hide cursor
          "\x1B[H"     // Move cursor to home (0, 0)
          "--------------------------------------------------------\x1B[0K\r\n"
          "Radio       : %4d, %4d, %4d, %4d\x1B[0K\r\n"
          "Trims       : %7.4fH %7.4fP %7.4fR\x1B[0K\r\n"
          "Gains   Yaw : %7.4fP %7.4fI %7.4fD\x1B[0K\r\n"
          " Pitch/Roll : %7.4fP %7.4fI %7.4fD\x1B[0K\r\n"
          "Orientation : %7.2fH %7.2fP %7.2fR\x1B[0K\r\n"
          "Angular vel.: %7.3fY %7.3fP %7.3fR\x1B[0K\r\n"
          "Target      : %7.2fH %7.2fP %7.2fR\x1B[0K\r\n"
          "Errors      : %7.4fY %7.4fP %7.4fR\x1B[0K\r\n"
          "Cumu. Err.  : %7.4fY %7.4fP %7.4fR\x1B[0K\r\n"
          "Correct Yaw : %7.4fP %7.4fI %7.4fD\x1B[0K\r\n"
          "      Pitch : %7.4fP %7.4fI %7.4fD\x1B[0K\r\n"
          "      Roll  : %7.4fP %7.4fI %7.4fD\x1B[0K\r\n"
          "Powers      : %5.1f%% %5.1f%%\x1B[0K\r\n"
          " (x%7.5f) : %5.1f%% %5.1f%%\x1B[0K\r\n"
          "Atmosphere  : %.2fC %.2fPa (=%5.2fm)\x1B[0K\r\n"
          "Batt volt.  : %.2fV\x1B[0K\r\n"
          "mspi        : %.2fms.\x1B[0K\r\n"
          "--------------------------------------------------------\x1B[0K\r\n"
          "%c > %s\x1B[0K"
          "\x1B[0J"     // Delete until end of screen
          "\x1B[?25h",  // Show cursor
          radio_rxValues[0],
          radio_rxValues[1],
          radio_rxValues[2],
          radio_rxValues[3],
          headingTrim,
          pitchTrim,
          rollTrim,
          y_proportionalGain,
          y_integralGain,
          y_derivativeGain,
          pr_proportionalGain,
          pr_integralGain,
          pr_derivativeGain,
          heading,
          pitch,
          roll,
          yawRate,
          pitchRate,
          rollRate,
          targetHeading,
          targetPitch,
          targetRoll,
          yawError,
          pitchError,
          rollError,
          pid_cumulativeYawError,
          pid_cumulativePitchError,
          pid_cumulativeRollError,
          proportionalYawCorrection,
          integralYawCorrection,
          derivativeYawCorrection,
          proportionalPitchCorrection,
          integralPitchCorrection,
          derivativePitchCorrection,
          proportionalRollCorrection,
          integralRollCorrection,
          derivativeRollCorrection,
          motor1Power_FL * 100.f,
          motor3Power_FR * 100.f,
          cosineLossCorrection,
          motor4Power_RL * 100.f,
          motor2Power_RR * 100.f,
          temperature,
          pressure,
          altitude,
          batt_mavgValue * BATT_VOLTAGE_MULTIPLIER / 4095.f,
          telemetry_mavgProcTime / 1000.f,
          "|/-\\|/-\\"[telemetry_iterationCount % 8],
          command_buffer
      );

      telemetry_lastTransmission_us = currentTick_us;
      telemetry_iterationCount++;
    }

    if (command_ready) { command_handle(); }

    int_fast32_t procTime = GetTickPrecise() - currentTick_us;
    if (telemetry_mavgProcTime == -1) {
      telemetry_mavgProcTime = procTime;
    } else {
      telemetry_mavgProcTime = (telemetry_mavgProcTime * 15 / 16) + (procTime / 16);
    }

    pid_lastLoopTime_us = currentTick_us;
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
  htim2.Init.Period = 4294967295;
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
  htim3.Init.Prescaler = 27 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000 - 1;
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
  sConfigOC.Pulse = 4000;
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
  huart6.Init.BaudRate = 460800;
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
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &RADIO_RX_UART) {
    uint16_t checksum = 0;
    for (int i = 0; i < 30; i++) { checksum += radio_rxBuffer[i]; }
    checksum += ((uint16_t)(radio_rxBuffer[31]) << 8) | radio_rxBuffer[30];
    if (checksum == 0xFFFF && radio_rxBuffer[0] == 0x20 && radio_rxBuffer[1] == 0x40) {
      for (int channel = 0; channel < 10; channel++) {
        uint16_t value = ((uint16_t)(radio_rxBuffer[channel * 2 + 3]) << 8) | radio_rxBuffer[channel * 2 + 2];
        radio_rxValues[channel] = value < 1000 ? 1000 : (value > 2000 ? 2000 : value);
      }
      radio_consecutiveErrors = 0;
      // Serial_Transmitln_DMA("Received and processed valid radio packet");
      // HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, HAL_GetTick() & (1 << 6) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
      radio_consecutiveErrors++;
      // Serial_Transmitf_DMA(
      //     "Received invalid radio packet. Checksum: 0x%04X, Length: 0x%02X, Protocol: 0x%02X\r\n",
      //     checksum,
      //     radio_rxBuffer[0],
      //     radio_rxBuffer[1]
      // );
      HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
      // HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, HAL_GetTick() & (1 << 6) ? GPIO_PIN_SET : GPIO_PIN_RESET);

      if (radio_consecutiveErrors >= RADIO_CONSECUTIVE_ERROR_THRESHOLD) {
        Serial_Transmitf_DMA(
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  uint_fast16_t size = Size;
  if (huart == &SERIAL_UART) {
    for (uint_fast16_t i = 0; i < size; i++) {
      char c = serial_rxBuffer[i];
      if (c >= 0x20 && c <= 0x7E) {
        command_buffer[command_bufferIndex++] = c;
        command_buffer[command_bufferIndex] = 0x00;
      } else if ((c == 0x08 || c == 0x7F) && command_bufferIndex > 0) {  // Backspace, DEL
        command_buffer[--command_bufferIndex] = 0x00;
      } else if ((c == 0x0A || c == 0x0D) && command_bufferIndex > 0) {  // LF
        command_ready = true;
      }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&SERIAL_UART, serial_rxBuffer, 32);
  } else if (huart == &WLSER_UART) {
    for (uint_fast16_t i = 0; i < size; i++) {
      char c = wlser_rxBuffer[i];
      if (c >= 0x20 && c <= 0x7E) {
        command_buffer[command_bufferIndex++] = c;
        command_buffer[command_bufferIndex] = 0x00;
      } else if ((c == 0x08 || c == 0x7F) && command_bufferIndex > 0) {  // Backspace, DEL
        command_buffer[--command_bufferIndex] = 0x00;
      } else if ((c == 0x0A || c == 0x0D) && command_bufferIndex > 0) {  // LF
        command_ready = true;
      }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&WLSER_UART, wlser_rxBuffer, 32);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  int_fast32_t tick = HAL_GetTick();
  if (GPIO_Pin == GPIO_PIN_13 && tick - interrupts_lastTriggers[13] > EXTI_DEBOUNCE) {
    override_outputDisabled = !override_outputDisabled;
    HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, override_outputDisabled);
    interrupts_lastTriggers[13] = tick;
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

  MOTOR_1_FL_PWM_CCR = 4000;
  MOTOR_2_RR_PWM_CCR = 4000;
  MOTOR_3_FR_PWM_CCR = 4000;
  MOTOR_4_RL_PWM_CCR = 4000;

  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);
  while (1) {
    HAL_GPIO_TogglePin(LED_Red_GPIO_Port, LED_Red_Pin);
    HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
    HAL_GPIO_TogglePin(LED_Blue_GPIO_Port, LED_Blue_Pin);
    DelayPrecise(125000);
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
