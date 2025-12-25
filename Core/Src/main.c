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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "CAN_receive.h"
#include "pid.h"
#include"bsp_can.h"
#include<stdio.h>
#include"bsp_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern rc_info_t rc;
char buf[200];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t motor1out;
int16_t motor2out;
int16_t motor3out;
int16_t motor4out;
uint16_t motor1_speed_rpm;
uint16_t motor2_speed_rpm;
uint16_t motor3_speed_rpm;
uint16_t motor4_speed_rpm;
int16_t targety_speed_rpm;
int16_t targetx_speed_rpm;
int16_t targetw_speed_rpm;
int16_t target1_speed_rpm;
int16_t target2_speed_rpm;
int16_t target3_speed_rpm;
int16_t target4_speed_rpm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  //dbus_uart_init();//DBUS遥控器启动模块
  pid_struct_t motor_pid;
  pid_init(&motor_pid, 40.0f, 0.15f, 1.00f, 5000.0f, 300.0f);  // pitch PID
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN2_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();//滤波器初始化
  dbus_uart_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1);
    const motor_measure_t *motor1=get_chassis_motor_measure_point(0);
    motor1_speed_rpm=motor1->speed_rpm;
    motor1_speed_rpm/=19;
    const motor_measure_t *motor2=get_chassis_motor_measure_point(1);
    motor2_speed_rpm=motor2->speed_rpm;
    motor2_speed_rpm/=19;
    const motor_measure_t *motor3=get_chassis_motor_measure_point(2);
    motor3_speed_rpm=motor3->speed_rpm;
    motor3_speed_rpm/=19;
    const motor_measure_t *motor4=get_chassis_motor_measure_point(3);
    motor4_speed_rpm=motor4->speed_rpm;
    motor4_speed_rpm/=19;
    //获取当前速度
    targetx_speed_rpm=rc.ch1;
    targety_speed_rpm=rc.ch2;
    targetw_speed_rpm=rc.sw1-rc.sw2;
    //遥控器数据获取
    target1_speed_rpm=-targetx_speed_rpm-targety_speed_rpm+targetw_speed_rpm;
    target2_speed_rpm=+targetx_speed_rpm-targety_speed_rpm+targetw_speed_rpm;
    target3_speed_rpm=+targetx_speed_rpm+targety_speed_rpm+targetw_speed_rpm;
    target4_speed_rpm=-targetx_speed_rpm+targety_speed_rpm+targetw_speed_rpm;
    //四个麦轮计算
    motor1out=(int16_t)pid_calc(&motor_pid,target1_speed_rpm,motor1_speed_rpm);
    motor2out=(int16_t)pid_calc(&motor_pid,target2_speed_rpm,motor2_speed_rpm);
    motor3out=(int16_t)pid_calc(&motor_pid,target3_speed_rpm,motor3_speed_rpm);
    motor4out=(int16_t)pid_calc(&motor_pid,target4_speed_rpm,motor4_speed_rpm);
    //PID计算（使用的pid.c经过改造可以实现处理角度环绕）
    CAN_cmd_chassis(motor1out,motor2out,motor3out,motor4out);
    //CAN_cmd_chassis(300,300,300,300);//测试用
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_SET);
    //LED为测试CAN协议用
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
