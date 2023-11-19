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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
//#include "HCSR04.h"
#include "tm_stm32_delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    White = 0,
    Black = 1
} LineColor;

typedef struct {
    volatile bool LeftTraValue;
    volatile bool CenterTraValue;
    volatile bool RightTraValue;
} InfraredValues;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FORWARD_SPEED			(70U)
#define ROTATE_SPEED 			(FORWARD_SPEED / 2)
#define CMD_LEN_1				(3U)
#define CMD_LEN_2				(4U)
#define CMD_LEN_6				(3U)
#define SERVO_ANGLE				(90U)
#define MAX_ATTEMPTS 			(3U)
#define AUTO_STOP_DELAY_TIME	(500U)
#define HCSR04_SENSOR1  		(0U)
#define PERIOD_SCAN_MOVE_CMD	(20U)
#define REPEAT					(1U)
#define ONCE					(0U)
#define IMMEDIATELY				(1U)
#define AFTER_START_API			(0U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//FORVARD			%F#
//BACKWARD			%B#
//LEFT				%L#
//RIGHT				%R#
//CAM_UP			%H#
//CAM_DOWN			%G#
//LINE_TRACKING		%T#
//OBJECT_AVOIDANCE	%A#
//OBJECT_FOLOWING	%Z#
//STOP				%S#
uint8_t rxBuffer_1[CMD_LEN_1] = {'\000'};
uint8_t rxBuffer_2[CMD_LEN_2] = {'\000'};
uint8_t rxBuffer_6[CMD_LEN_6] = {'\000'};
volatile uint8_t cmd = '\000';
volatile bool flagCmdCompletion = false;
volatile bool flagDirect = false;
uint8_t servoAngle = SERVO_ANGLE;
uint8_t bitIndex;
uint8_t cmdli;
uint32_t code;
uint32_t tempCode;
InfraredValues infrared;
float Distance = 0.0;
TM_DELAY_Timer_t* pTimerScanMoveCmd;
TM_DELAY_Timer_t* pTimerAutoStop;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Move_Forward(uint32_t car_speed);
void Move_Backward(uint32_t car_speed);
void Rotate_Left(uint32_t car_speed);
void Rotate_Right(uint32_t car_speed);
void Move_Left(uint32_t car_speed);
void Move_Right(uint32_t car_speed);
void Move_Stop();
uint8_t ConvertCode (uint32_t code);
int map(int st1, int fn1, int st2, int fn2, int value);
void ServoWrite(int angle);
void InfraredTracing(InfraredValues* infrared);
void ReadSensors(InfraredValues* infrared);
void ScanMoveCmd(struct _TM_DELAY_Timer_t* my_timer, void *parameters);
void AutoStopMove(struct _TM_DELAY_Timer_t* my_timer, void *parameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM11_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  printf("WiFi Robot Car is Started\r\n");
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_UART_Receive_DMA(&huart1, rxBuffer_1, CMD_LEN_1);
  HAL_UART_Receive_DMA(&huart2, rxBuffer_2, CMD_LEN_2);
  HAL_UART_Receive_DMA(&huart6, rxBuffer_6, CMD_LEN_6);
  HAL_TIM_Base_Start(&htim11);
  __HAL_TIM_SET_COUNTER(&htim11, 0);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  ServoWrite(90);
  TM_DELAY_Init();
  pTimerScanMoveCmd	= TM_DELAY_TimerCreate(PERIOD_SCAN_MOVE_CMD, REPEAT, IMMEDIATELY, ScanMoveCmd, NULL);
  pTimerAutoStop = TM_DELAY_TimerCreate(AUTO_STOP_DELAY_TIME, ONCE, AFTER_START_API, AutoStopMove, NULL);
//  HCSR04_Init(HCSR04_SENSOR1, &htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HCSR04_Trigger(HCSR04_SENSOR1);
//	  HAL_Delay(15);
//	  Distance = HCSR04_Read(HCSR04_SENSOR1);
//	  printf("Distance = %d cm\r\n", (int) Distance);
//	  HAL_Delay(1000);
	if (flagCmdCompletion)
	{
	  flagCmdCompletion = false;
	  printf("CMD: %c\r\n", cmd);
	  switch (cmd)
	  {
			case 'F':
				Move_Forward((uint32_t) FORWARD_SPEED);
				break;
			case 'B':
				Move_Backward((uint32_t) FORWARD_SPEED);
				break;
			case 'L':
				Rotate_Left((uint32_t) ROTATE_SPEED);
				break;
			case 'R':
				Rotate_Right((uint32_t) ROTATE_SPEED);
				break;
			case 'S':
				Move_Stop();
				break;
			case 'H':
				servoAngle = servoAngle + 4;
				if (servoAngle >= 180)
				{
					servoAngle = 180;
				}
				ServoWrite(servoAngle);
				break;
			case 'G':
				servoAngle = servoAngle - 4;
				if (servoAngle <= 0)
				{
					servoAngle = 0;
				}
				ServoWrite(servoAngle);
				break;
			case 'T':
				InfraredTracing(&infrared);
			default:
				Move_Stop();
				break;
	  }
		  cmd = '\000';
	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == D3_Pin)
	{
		if (__HAL_TIM_GET_COUNTER(&htim11) > 8000)
		{
		  tempCode = 0;
		  bitIndex = 0;
		}
		else if (__HAL_TIM_GET_COUNTER(&htim11) > 1700)
		{
		  tempCode |= (1UL << (31-bitIndex));   // write 1
		  bitIndex++;
		}
		else if (__HAL_TIM_GET_COUNTER(&htim11) > 1000)
		{
		  tempCode &= ~(1UL << (31-bitIndex));  // write 0
		  bitIndex++;
		}
		if(bitIndex == 32)
		{
		  cmdli = ~tempCode; // Logical inverted last 8 bits
		  cmd = tempCode >> 8; // Second last 8 bits

		  if(cmdli == cmd) // Check for errors
		  {
			code = tempCode; // If no bit errors
			printf("IrDA cmd: %#lx\r\n", code);
			cmd = ConvertCode(code);
		  }
		  bitIndex = 0;
		}
		  __HAL_TIM_SET_COUNTER(&htim11, 0);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		if (('%' == rxBuffer_1[0]) && ('#' == rxBuffer_1[2]))
		{
			cmd = rxBuffer_1[1];
			rxBuffer_1[0] = '\000';
			rxBuffer_1[1] = '\000';
			rxBuffer_1[2] = '\000';
			flagCmdCompletion = true;
		}
	}

	if (huart->Instance == USART2)
	{
		if (('%' == rxBuffer_2[0]) && ('#' == rxBuffer_2[2]))
		{
			cmd = rxBuffer_2[1];
			rxBuffer_2[0] = '\000';
			rxBuffer_2[1] = '\000';
			rxBuffer_2[2] = '\000';
			flagCmdCompletion = true;
		}
	}

	if (huart->Instance == USART6)
	{
		if (('%' == rxBuffer_6[0]) && ('#' == rxBuffer_6[2]))
		{
			cmd = rxBuffer_6[1];
			rxBuffer_6[0] = '\000';
			rxBuffer_6[1] = '\000';
			rxBuffer_6[2] = '\000';
			flagCmdCompletion = true;
			pTimerAutoStop = TM_DELAY_TimerAutoReloadValue(pTimerAutoStop, AUTO_STOP_DELAY_TIME);
			pTimerAutoStop = TM_DELAY_TimerStart(pTimerAutoStop);
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
//		HCSR04_TMR_IC_ISR(htim);
	}
}

// Interrupt Handling Function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM10)
	{
		HAL_TIM_Base_Stop_IT(htim);

	}

	if (htim->Instance == TIM2)
	{
//		HCSR04_TMR_OVF_ISR(htim);
	}
}

void Move_Forward(uint32_t car_speed)
{
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
	int pulse = map(0, 100, 0, 999, (int) car_speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t) pulse);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t) pulse);
}

void Move_Backward(uint32_t car_speed)
{
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
	int pulse = map(0, 100, 0, 999, (int) car_speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t) pulse);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t) pulse);
}

void Rotate_Left(uint32_t car_speed)
{
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
	int pulse = map(0, 100, 0, 999, (int) car_speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t) pulse);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t) pulse);
}

void Rotate_Right(uint32_t car_speed)
{
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
	int pulse = map(0, 100, 0, 999, (int) car_speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t) pulse);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t) pulse);
}

void Move_Left(uint32_t car_speed)
{
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
	int pulse = map(0, 100, 0, 999, (int) car_speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t) 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t) pulse);
}

void Move_Right(uint32_t car_speed)
{
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
	int pulse = map(0, 100, 0, 999, (int) car_speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t) pulse);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t) 0);
}

void Move_Stop()
{
//	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
}

uint8_t ConvertCode (uint32_t code)
{
	uint8_t cmd = '\000';
	switch (code)
	{
		case (0xFFA25D):
			flagCmdCompletion = true;
			break;

		case (0xFF629D):
			cmd = 'F';
			flagCmdCompletion = true;
			break;

		case (0xFFE21D):
			flagCmdCompletion = true;
			break;

		case (0xFF22DD):
			cmd = 'L';
			flagCmdCompletion = true;
			break;

		case (0xFF02FD):
			cmd = 'S';
			flagCmdCompletion = true;
			break;

		case (0xFFC23D):
			cmd = 'R';
			flagCmdCompletion = true;
			break;

		case (0xFFE01F):
			flagCmdCompletion = true;
			break;

		case (0xFFA857):
			cmd = 'B';
			flagCmdCompletion = true;
			break;

		case (0xFF906F):
			flagCmdCompletion = true;
			break;

		case (0xFFB04F):
			flagCmdCompletion = true;
			break;

		case (0XFF6897):
			flagCmdCompletion = true;
			break;

		case (0xFF9867):
			cmd = 'H';
			flagCmdCompletion = true;
			break;

		case (0xFF38C7):
			cmd = 'G';
			flagCmdCompletion = true;
			break;

		case (0xFF18E7):
			flagCmdCompletion = true;
			break;

		case (0xFF10EF):
			flagCmdCompletion = true;
			break;

		case (0xFF5AA5):
			flagCmdCompletion = true;
			break;

		case (0xFF4AB5):
			flagCmdCompletion = true;
			break;

		case (0xFF30CF):
			cmd = 'T';
			flagCmdCompletion = true;
			break;
		default :
			break;
	}
	return cmd;
}

int map(int st1, int fn1, int st2, int fn2, int value)
{
    return (int)((float)(value - st1) * (fn2 - st2) / (float)(fn1 - st1) + st2);
}

void ServoWrite(int angle)
{
	htim4.Instance->CCR1 = map(0, 180, 50, 250, angle);
}

void ReadSensors(InfraredValues* infrared)
{
	infrared->LeftTraValue = HAL_GPIO_ReadPin(A1_GPIO_Port, A1_Pin);
    infrared->CenterTraValue = HAL_GPIO_ReadPin(D7_GPIO_Port, D7_Pin);
    infrared->RightTraValue = HAL_GPIO_ReadPin(D8_GPIO_Port, D8_Pin);
}

void InfraredTracing(InfraredValues* infrared)
{
	while(cmd == 'T')
	{
		ReadSensors(infrared);

		if (infrared->LeftTraValue == White && (infrared->CenterTraValue == Black && infrared->RightTraValue == White))
		{
			Move_Forward(FORWARD_SPEED);
		}
		else if (infrared->LeftTraValue == Black && (infrared->CenterTraValue == Black && infrared->RightTraValue == White))
		{
			Rotate_Left((uint32_t) ROTATE_SPEED - 10U);
		}
		else if (infrared->LeftTraValue == Black && (infrared->CenterTraValue == White && infrared->RightTraValue == White))
		{
			Rotate_Left(ROTATE_SPEED);
		}
		else if (infrared->LeftTraValue == White && (infrared->CenterTraValue == White && infrared->RightTraValue == Black))
		{
			Rotate_Right(ROTATE_SPEED);
		}
		else if (infrared->LeftTraValue == White && (infrared->CenterTraValue == Black && infrared->RightTraValue == Black))
		{
			Rotate_Right((uint32_t) ROTATE_SPEED - 10U);
		}
		else if (infrared->LeftTraValue == Black && (infrared->CenterTraValue == Black && infrared->RightTraValue == Black))
		{
			pTimerAutoStop = TM_DELAY_TimerAutoReloadValue(pTimerAutoStop, AUTO_STOP_DELAY_TIME * 3);
			pTimerAutoStop = TM_DELAY_TimerStart(pTimerAutoStop);

			while (infrared->LeftTraValue == Black && (infrared->CenterTraValue == Black && infrared->RightTraValue == Black))
			{
				ReadSensors(infrared);

				if(flagDirect)
				{
					Move_Forward(ROTATE_SPEED);
				}
				else
				{
					Rotate_Left(ROTATE_SPEED);
				}
			}
			flagDirect = !flagDirect;
		}
	}
}

void ScanMoveCmd(struct _TM_DELAY_Timer_t* my_timer, void *parameters)
{

}

void AutoStopMove(struct _TM_DELAY_Timer_t* my_timer, void *parameters)
{
	flagDirect = !flagDirect;
	Move_Stop();
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
