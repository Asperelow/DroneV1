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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MotorControl.h"
#include "PID.h"
#include "MPL3115A2.h"
#include "icm20948.h"
#include "orientationFilter.h"


// Gyro, Accelerometer, and Magnometer

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Calibrate 	1
#define ICM20948_SPI					(&hspi1)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROLLER_RECEIVE_SIGNAL_MS 100

#define LPF_GYR_ALPHA 					0.01f
#define LPF_ACC_ALPHA 					0.10f

#define GYRO_PREDICT_RATE_MS 			10		// This is not used at the moment - To be used when Kalman is implemented
#define GYRO_UPDATE_RATE_MS 			5
#define BARO_UPDATE_RATE_MS 			100
#define MOTOR_UPDATE_RATE_MS 			50
#define DESIRED_POSITION_UPDATE_RATE_MS	500
#define RAD_TO_DEG 						57.2957795131f


/*
#define KALMAN_P_INIT 0.1f
#define KALMAN_Q 0.001f
#define KALMAN_R 0.011f
#define KALMAN_PREDICT_PERIOD_MS 10
#define KALMAN_UPDATE_PERIOD_MS 100
*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t prevAHRSUpdateTime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



int updatePerceptionAlgorithm = 0;	// Perceive block trigger 	- Sensor Inputs
int updatePlanningAlgorithm = 0;	// Plan block trigger 		- PID Updates

uint32_t flyskyPWMFrequencyCh1, flyskyPWMDutyCycleCh1;
uint32_t flyskyPWMFrequencyCh2, flyskyPWMDutyCycleCh2;
uint32_t flyskyPWMFrequencyCh3, flyskyPWMDutyCycleCh3;
uint32_t flyskyPWMFrequencyCh4, flyskyPWMDutyCycleCh4;
uint32_t captureValue;
int prevControllerUpdateTime;

// ################## Controller PWM Interrupt Capture Callback ###############################
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	/*
	 * Captures the PWM input on various channels and stores the result as an in from 50:100 for
	 * the flySky controller. Other PWM signals will likely land outside this range.
	 */
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		captureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		if(captureValue){
			if(htim9.Channel == HAL_TIM_ACTIVE_CHANNEL_1){
				flyskyPWMFrequencyCh1 = SystemCoreClock / (captureValue);
				flyskyPWMDutyCycleCh1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) / 305;
			} else if(htim3.Channel == HAL_TIM_ACTIVE_CHANNEL_1){
				flyskyPWMFrequencyCh2 = SystemCoreClock / (captureValue);
				flyskyPWMDutyCycleCh2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) / 305;
			} else if(htim4.Channel == HAL_TIM_ACTIVE_CHANNEL_1){
				flyskyPWMFrequencyCh3 = SystemCoreClock / (captureValue);
				flyskyPWMDutyCycleCh3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) / 305;
			} else if(htim5.Channel == HAL_TIM_ACTIVE_CHANNEL_1){
				flyskyPWMFrequencyCh4 = SystemCoreClock / (captureValue);
				flyskyPWMDutyCycleCh4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) / 305;
			}
		}
	}
}

DroneMotorCommand MotorCommands;
PIDController zPID;
PIDController RollPID;
PIDController PitchPID;
PIDController yawPID;


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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // ############## Controller Input Initialize #########################################
  float controllerLeftX, controllerRightX, controllerRightY, controllerLeftY;

  // ############## Gyro, Accel & Mag Initialize ########################################
  icm20948_init();
  ak09916_init();

  axises my_gyro;
  axises my_accel;
  axises my_mag;

  uint32_t prevGyroUpdateTime = 0;

  // Initialize collaborative Filter
  OrientationEuler orientation;
  orientation_filter_init(0.98f);


  // ############## Init Barometer ######################################################
  uint32_t prevBaroUpdateTime = 0;
  MPL3115A2_DataTypeDef barometerData;

  // Initialize MPL3115A2 sensor
  MPL3115A2_Init(&hi2c1);
  // Get first sample to initialize barometer filter
  while(barometerData.pressure < 10 || barometerData.pressure > 1600){
  //while(barometerData.pressure == 0){
  	  barometerData = MPL3115A2_ReadData(&hi2c1);
	  prevBaroUpdateTime = HAL_GetTick();
	  // !!! Add code to indicate barometer startup loop - LED
	  HAL_Delay(200);
  }

  // ------------ Filter Initialize ----------------------------------------------------
  float currentAltitude = barometerData.pressure;
  float initAltitude = barometerData.pressure;

  altitude_filter_update(&currentAltitude, &barometerData.pressure, &my_accel.z);


  // #################### Start Timers that control Command #####################################
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // ############## Init  Motor Command Parameters ######################################
  MotorCommands.throttleMin = 200;
  MotorCommands.throttleMax = 500;
  MotorCommands.TimARR = 1000;

  // #################### Calibrate (if first time using motor) #################################
  #if Calibrate
  	TIM1->CCR1 = MotorCommands.throttleMax;
  	TIM1->CCR2 = MotorCommands.throttleMax;
  	TIM1->CCR3 = MotorCommands.throttleMax;
  	TIM1->CCR4 = MotorCommands.throttleMax;
  	HAL_Delay(2000);

  	TIM1->CCR1 = MotorCommands.throttleMin;
  	TIM1->CCR2 = MotorCommands.throttleMin;
  	TIM1->CCR3 = MotorCommands.throttleMin;
  	TIM1->CCR4 = MotorCommands.throttleMin;
  	HAL_Delay(1000);

  #endif

  // ############### Init z/Roll/Pitch/Yaw PID Parameters ###############################
  float desiredZ = initAltitude;
  float desiredRoll, desiredPitch, desiredYaw;
  prevControllerUpdateTime = 0;

  // -------------- PID - z --------------
  PIDController_Init(&zPID);
  zPID.T = GYRO_UPDATE_RATE_MS;
  uint32_t prevzUpdateTime = 0;

  zPID.K_p = 1;
  zPID.K_i = 0.0;
  zPID.K_d = 0;
  zPID.tau = 0;

  zPID.limMin = -100;
  zPID.limMax = 100;

  // -------------- PID - Roll --------------
  PIDController_Init(&RollPID);
  RollPID.T = GYRO_UPDATE_RATE_MS;
  uint32_t prevRollUpdateTime = 0;

  RollPID.K_p = 1;
  RollPID.K_i = 0.0;
  RollPID.K_d = 0;
  RollPID.tau = 0;

  RollPID.limMin = -100;
  RollPID.limMax = 100;


  // -------------- PID - Pitch --------------
  PIDController_Init(&PitchPID);
  PitchPID.T = GYRO_UPDATE_RATE_MS;
  uint32_t prevPitchUpdateTime = 0;

  PitchPID.K_p = 1;
  PitchPID.K_i = 0;
  PitchPID.K_d = 0;
  PitchPID.tau = 0;

  PitchPID.limMin = -100;
  PitchPID.limMax = 100;

  // -------------- PID - Yaw --------------
  PIDController_Init(&yawPID);
  yawPID.T = GYRO_UPDATE_RATE_MS;
  uint32_t prevYawUpdateTime = 0;

  yawPID.K_p = 1;
  yawPID.K_i = 0.0;
  yawPID.K_d = 0;
  yawPID.tau = 0;

  yawPID.limMin = -100;
  yawPID.limMax = 100;

  // ############## Set Motor Commands to Minimum on startup ############################
  uint32_t prevMotorUpdateTime = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// ############################ Sense ############################
	// -------------- Read Gyro, Accel, and Mag Sensor --------------
	controllerLeftX = 2.0f * ((float)flyskyPWMDutyCycleCh1 - 75);
	controllerRightX = 2.0f * ((float)flyskyPWMDutyCycleCh2 - 75);
	controllerRightY = 2.0f * ((float)flyskyPWMDutyCycleCh3 - 75);
	controllerLeftY = 2.0f * ((float)flyskyPWMDutyCycleCh4 - 75);

	if(controllerLeftX < -50){
		controllerLeftX = -50;
	} else if(controllerLeftX > 50){
		controllerLeftX = 50;
	}

	if(controllerLeftY < -50){
		controllerLeftY = -50;
	} else if(controllerLeftY > 50){
		controllerLeftY = 50;
	}

	if(controllerRightX < -50){
		controllerRightX = -50;
	} else if(controllerRightX > 50){
		controllerRightX = 50;
	}

	if(controllerRightY < -50){
		controllerRightY = -50;
	} else if(controllerRightY > 50){
		controllerRightY = 50;
	}

	// -------------- Read Gyro, Accel, and Mag Sensor --------------
	if((HAL_GetTick() - prevGyroUpdateTime) >= GYRO_UPDATE_RATE_MS){
		icm20948_gyro_read_dps(&my_gyro);
		icm20948_accel_read_g(&my_accel);
		ak09916_mag_read_uT(&my_mag);

		my_gyro.x = my_gyro.x * 0.017;
		my_gyro.y = my_gyro.y * 0.017;
		my_gyro.z = my_gyro.z * 0.017;

		my_accel.x = my_accel.x  * 10;
		my_accel.y = my_accel.y *  10;
		my_accel.z = my_accel.z *  10;

        orientation_filter_update(my_accel.x, my_accel.y, my_accel.z, my_gyro.x, my_gyro.y, GYRO_UPDATE_RATE_MS*0.001f);
        orientation = orientation_filter_get_orientation();
        orientation.pitch = -orientation.pitch;

		prevGyroUpdateTime = HAL_GetTick();
	}

	// -------------- Read Barometric Pressure Sensor --------------
	if((HAL_GetTick() - prevBaroUpdateTime) >= BARO_UPDATE_RATE_MS){
		barometerData = MPL3115A2_ReadData(&hi2c1);
        altitude_filter_update(&currentAltitude, &barometerData.pressure, &my_accel.z);

		prevBaroUpdateTime = HAL_GetTick();

	}

	// ############################ Perceive ############################
	if((HAL_GetTick() - prevControllerUpdateTime) >= DESIRED_POSITION_UPDATE_RATE_MS){
		desiredZ = desiredZ + 0.01f * controllerLeftY;
		desiredYaw = desiredYaw + (0.001f * controllerLeftX);
		desiredRoll = 0.01f * controllerRightX;
		desiredPitch = 0.01f * controllerRightY;
		prevControllerUpdateTime = HAL_GetTick();
	}


	// ############################ Plan ############################
	// Need to add commands to change setpoints given various inputs from user
	if((HAL_GetTick() - prevzUpdateTime) >= zPID.T){
		PIDController_Update(&zPID, desiredZ, currentAltitude);
		prevzUpdateTime = HAL_GetTick();
	}
	if((HAL_GetTick() - prevRollUpdateTime) >= zPID.T){
		//PIDController_Update(&RollPID, desiredRoll, currentOrientation.q1);
		prevRollUpdateTime = HAL_GetTick();
	}
	if((HAL_GetTick() - prevPitchUpdateTime) >= zPID.T){
		//PIDController_Update(&PitchPID, desiredPitch, currentOrientation.q2);
		prevPitchUpdateTime = HAL_GetTick();
	}
	if((HAL_GetTick() - prevYawUpdateTime) >= zPID.T){
		//PIDController_Update(&yawPID, desiredYaw, currentOrientation.q3);
		prevYawUpdateTime = HAL_GetTick();
	}

	// ############################ Act ############################
	if((HAL_GetTick() - prevMotorUpdateTime) >= MOTOR_UPDATE_RATE_MS){
		MotorCommands.z = zPID.out;

		Update_Duty_Cycles(&MotorCommands);

		TIM1->CCR1 = MotorCommands.M1DutyCycle;
		TIM1->CCR2 = MotorCommands.M2DutyCycle;
		TIM1->CCR3 = MotorCommands.M3DutyCycle;
		TIM1->CCR4 = MotorCommands.M4DutyCycle;

		prevMotorUpdateTime = HAL_GetTick();
	}

	// #############################################################

	if(currentAltitude - 10 < desiredZ && desiredZ < currentAltitude + 10){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
	} else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
