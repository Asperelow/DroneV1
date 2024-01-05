/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "icm20948.h"

#include "kalmanFilters.h"
#include "MotorControl.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Calibrate 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LPF_GYR_ALPHA 0.01f
#define LPF_ACC_ALPHA 0.10f

#define KALMAN_P_INIT 0.1f
#define KALMAN_Q 0.001f
#define KALMAN_R 0.011f

#define KALMAN_PREDICT_PERIOD_MS 10
#define KALMAN_UPDATE_PERIOD_MS 100

#define RAD_TO_DEG 57.2957795131f


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

MPU6050_t MPU6050;
KalmanRollPitch ekf;
DroneMotorCommand MotorCommands;
PIDController RollPID;
PIDController PitchPID;

// New gyro variables
axises my_gyro;
axises my_accel;
axises my_mag;

float tempKalAccel[3];
float tempKalVel[3];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Check if interrupt pin has been fired
	if(GPIO_Pin == IMU_INTERRUPT_Pin)
	{
		MPU6050.rx = 1;
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init(&hi2c1, &MPU6050);


  float KalmanQ[2] = {KALMAN_Q, KALMAN_Q};
  float KalmanR[3] = {KALMAN_R, KALMAN_R, KALMAN_R};

  kalman_roll_pitch_init(&ekf, KALMAN_P_INIT, KalmanQ, KalmanR);

  // Initialize timers
  uint32_t timerKalmanPredict = 0;
  uint32_t timerKalmanUpdate = 0;

  // New Gyro
  uint32_t timerGyroUpdateInterval = 0;

  uint32_t tim1ch2 = 0;

  // Init PID controller for roll and pitch;
  PIDController_Init(&RollPID);
  PIDController_Init(&PitchPID);

  RollPID.T = KALMAN_UPDATE_PERIOD_MS;
  PitchPID.T = KALMAN_UPDATE_PERIOD_MS;

  RollPID.K_p = 100;
  RollPID.K_i = 0.2;
  RollPID.K_d = 0;
  RollPID.tau = 0;

  RollPID.limMin = -100;
  RollPID.limMax = 100;


  PitchPID.K_p = 20;
  PitchPID.K_i = 0;
  PitchPID.K_d = 0;
  PitchPID.tau = 0;

  PitchPID.limMin = -1000;
  PitchPID.limMax = 1000;

  // ---------- Motor & ESC Setup ---------------------
  TIM1->CCR2 = 250; // ARR = 1000
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);


  /*
   * This is code for 4 motor control, return to it once one motor is working
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  //TIM2->CCR1 = 200; // ARR = 1000
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  //TIM2->CCR2 = 400; // ARR = 1000
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  //TIM2->CCR3 = 600; // ARR = 1000
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  //TIM2->CCR4 = 800; // ARR = 1000
*/

  // Move this to an init function in MotorControl
#if Calibrate
  TIM1->CCR2 = 1000;
  HAL_Delay(3000);
  TIM1->CCR2 = 500;
  HAL_Delay(2000);
  TIM1->CCR2 = 0;

  MotorCommands.TimARR = 1000;
  MotorCommands.sensorMax = 100;
#endif

  //New Gyro
  icm20948_init();
  ak09916_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	//if((HAL_GetTick() - tim1ch2) >= 1000){
	//	TIM1->CCR2 += 5;
	//	if(TIM1->CCR2 >= 900){
	//		TIM1->CCR2 = 50;
	//	}
	//	tim1ch2 += 2000;
	//}

	if((HAL_GetTick() - timerGyroUpdateInterval) >= KALMAN_PREDICT_PERIOD_MS){
		icm20948_accel_read_g(&my_accel);
		ak09916_mag_read(&my_mag);

		float someAccelX = my_accel.x;
		float someAccelY = my_accel.y;

		float someMagX = my_mag.x;
		float someMagY = my_mag.y;
		float someMagZ = my_mag.z;

		if (my_accel.y > 0.5){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
		} else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
		}
	}
	// Update Gyro and Accelerometer values if IMU receive flag is triggered
	if(MPU6050.rx == 1){
		MPU6050_Read_Gyro(&hi2c1, &MPU6050);
		MPU6050_Read_Accel(&hi2c1, &MPU6050);
		float phi_rad = ekf.phi_rad;
		float theta_rad = ekf.theta_rad;


		float Drone_X_Pos = MPU6050.Accel[0];
		/*
		if (MPU6050.Accel[0] > 0.5){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
		}
		*/
	}

	if((HAL_GetTick() - timerKalmanPredict) >= KALMAN_PREDICT_PERIOD_MS){
		kalman_roll_pitch_predict(&ekf, &MPU6050.Gyro, 0.001f * KALMAN_PREDICT_PERIOD_MS);
		timerKalmanPredict += KALMAN_PREDICT_PERIOD_MS;
	}

	if((HAL_GetTick() - timerKalmanUpdate) >= KALMAN_UPDATE_PERIOD_MS){
		tempKalAccel[0] = my_accel.x;
		tempKalAccel[1] = my_accel.y;
		tempKalAccel[2] = my_accel.z;

		kalman_roll_pitch_update(&ekf, &tempKalAccel);
		timerKalmanUpdate += KALMAN_UPDATE_PERIOD_MS;


		float DroneRollPosition = ekf.phi_rad; //(ekf.phi_rad * 100);
		float DronePitchPosition = ekf.theta_rad; //(ekf.theta_rad * 100);


		MotorCommands.Roll = PIDController_Update(&RollPID, 0.0f, DroneRollPosition);
		float PIDout = MotorCommands.Roll;

		Update_Duty_Cycles(&MotorCommands);

		TIM1->CCR2 = MotorCommands.M1DutyCycle;
		int test = MotorCommands.M1DutyCycle;
	}


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
