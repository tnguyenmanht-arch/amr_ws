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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>          /* snprintf cho debug output */
#include "motor_driver.h"
#include "servo_buslinker.h"
#include "jetson_comm.h"
#include "ackermann.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Hằng số chuyển đổi cmd_vel nằm trong ackermann.c (CALC_Ackermann) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t last_odom_ms = 0;     /* Mốc thời gian gửi odom gần nhất */
static float    current_steer = 0.0f; /* Góc lái hiện tại (độ) để gửi lên Jetson */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void on_cmd_vel(float linear, float angular);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);          // Đủ để các module boot, không trigger motor timeout
  DRV_Motor_Init();        // Không check return — tiếp tục dù config fail
  DRV_Servo_Init();        // Servo về vị trí trung tâm (lái thẳng)
  APP_Comm_Init(on_cmd_vel); // Bật UART2 nhận lệnh "$VEL" từ Jetson

  /* ===== Chẩn đoán: scan bus + probe đọc voltage 0x00. Xóa sau khi xong. */
  DRV_Motor_ScanBus();
  DRV_Motor_ProbeVoltage();
  {
      char pbuf[80];
      snprintf(pbuf, sizeof(pbuf),
               "SCAN:found=0x%02X,count=%u | PROBE:volt=%u,rc=%u,ec=0x%lX\r\n",
               (unsigned)g_i2c_scan_found, (unsigned)g_i2c_scan_count,
               (unsigned)g_probe_volt, (unsigned)g_probe_rc,
               (unsigned long)g_last_enc_i2c_ec);
      APP_Comm_DebugPrint(pbuf);
  }
  /* ===== HẾT chẩn đoán ===== */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* ---- Gửi odometry lên Jetson định kỳ mỗi 10ms (100 Hz) ---- */
	  uint32_t now = HAL_GetTick();
	  if (now - last_odom_ms >= 10U) {
		  last_odom_ms = now;

		  /* Đọc encoder 2 bánh */
		  int32_t enc_l = 0, enc_r = 0;
		  DRV_Motor_GetEncoder(&enc_l, &enc_r);

		  /* ===== DEBUG — báo lỗi đọc I2C encoder. Xóa sau khi chẩn đoán xong. */
		  if (g_last_enc_i2c_err != 0) {
			  char errdbg[48];
			  snprintf(errdbg, sizeof(errdbg), "ENCERR:%c%d,ec=0x%lX\r\n",
					   g_last_enc_i2c_step, (int)g_last_enc_i2c_err,
					   (unsigned long)g_last_enc_i2c_ec);
			  APP_Comm_DebugPrint(errdbg);
		  }
		  /* ===== HẾT DEBUG ===== */

		  /* Gửi "$ODO,enc_l,enc_r,steer\n" lên Jetson */
		  APP_Comm_SendOdom(enc_l, enc_r, current_steer);
	  }

	  /* ---- Xử lý lệnh "$VEL" nhận từ Jetson (gọi on_cmd_vel khi đủ frame) ---- */
	  APP_Comm_Parse();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Callback nhận lệnh cmd_vel từ Jetson qua UART2.
 *         Gọi tự động từ APP_Comm_Parse() khi parse đủ 1 lệnh "$VEL" hợp lệ.
 * @note   Quy đổi Twist -> Ackermann nằm trong CALC_Ackermann (ackermann.c).
 *         Kiểm tra chiều quay motor và chiều lái servo trước khi chạy tự động.
 */
static void on_cmd_vel(float linear, float angular)
{
    int8_t spd_l, spd_r;
    /* Quy đổi cmd_vel -> tốc độ 2 bánh + góc lái; lưu góc lái để gửi odom */
    CALC_Ackermann(linear, angular, &spd_l, &spd_r, &current_steer);

    /* ===== DEBUG — xác nhận callback chạy & giá trị quy đổi. Xóa sau khi test. */
    char dbg[64];
    snprintf(dbg, sizeof(dbg), "DBG:cmd=%.2f,%.2f spd=%d steer=%.1f\r\n",
             (double)linear, (double)angular, (int)spd_l, (double)current_steer);
    APP_Comm_DebugPrint(dbg);
    /* ===== HẾT DEBUG ===== */

    DRV_Motor_SetSpeed(spd_l, spd_r);
    DRV_Servo_SetAngle(current_steer);
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
