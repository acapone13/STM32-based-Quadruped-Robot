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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pca9685.h"
//#include "uart_dma.h"
#include "spider.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_STRLEN	40
#define RX_BFR_SIZE	10
#define UARTn		1

//UARTDMA_HandleTypeDef huartdma;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Spider Configuration */
float refPos[4][3] = \
	{{58.33, 58.33, -77.5},{58.33, 58.33, -77.5},{0, 82.5, -77.5},{0, 82.5, -77.5}};

uint16_t anglePos[SERVO_COUNT]; //	Initialized to 0

typedef enum {Ref, NoRef}refVar;
typedef enum
{
	HOME,
	WALKING_FORWARD,
	WALKING_BACKWARD,
	WALKING_LEFT,
	WALKING_RIGHT,
	TURNING_RIGHT,
	TURNING_LEFT,
	MOVING_LEG,
	MOVING_JOINT,
	SIT,
	STAND,
	STILL
}spiderState;

typedef enum {Manual, Auto}spiderMode;

refVar REF;
spiderState state;

/* UART Related Variables */
char USART_TxStream[MAX_STRLEN];
uint8_t USART_RxStream[RX_BFR_SIZE];

uint8_t RxRollover = 0;
uint8_t RxCounter = 0;
uint8_t RxBfrPos = 0;

char command[MAX_STRLEN];
uint8_t flagRx = 0; uint8_t indRx = 0;
int count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
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
	REF = NoRef;
	state = STILL;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
//  UARTDMA_Init(&huartdma, &huart1);
  PCA9685_Init(&hi2c1);

  sprintf(USART_TxStream, "SPIDER ROBOT v1.2\n\r");

//  HAL_UART_Transmit(&huart1, (uint8_t*) USART_TxStream, (sizeof(USART_TxStream)-1), 100);
  HAL_UART_Transmit_DMA(&huart1, (uint8_t*) USART_TxStream, (uint16_t)(sizeof(USART_TxStream)-1));
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, USART_RxStream, RX_BFR_SIZE);

  // Start Timer 2 andTimer3
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (REF == Ref)
	  {
		  switch (state)
		  {
		  case WALKING_FORWARD:
			  forward_walk();
			  break;
		  case WALKING_BACKWARD:
			  backward_walk();
			  break;
		  case WALKING_RIGHT:
			  right_walk();
			  break;
		  case WALKING_LEFT:
			  left_walk();
			  break;
		  case TURNING_RIGHT:
			  turn_right();
			  break;
		  case TURNING_LEFT:
			  turn_left();
			  break;
		  case SIT:
			  sit();
			  break;
		  case STAND:
			  stand();
			  REF = NoRef;
			  break;
		  default:
			  break;
		  }
	  }
	  else
	  {
		  // Force Homing
		  homing(refPos);
		  state = HOME;
		  REF = Ref;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* I2C1_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/* USER CODE BEGIN 4 */
//	Utility Functions
void getState(spiderState currState)
{
	const char sTable[][MAX_STRLEN] = {
			"HOME",
			"WALKING_FORWARD",
			"WALKING_BACKWARD",
			"WALKING_LEFT",
			"WALKING_RIGHT",
			"TURNING_RIGHT",
			"TURNING_LEFT",
			"MOVING_LEG",
			"MOVING_JOINT",
			"SIT",
			"STAND",
			"STILL"
	};
	memset(&USART_TxStream[0], 0, sizeof(USART_TxStream));
	sprintf(USART_TxStream, "Current State: %s\n\r", sTable[currState]);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*) USART_TxStream, (uint16_t)(sizeof(USART_TxStream)-1));
}

//	Command Interpretation function
void InterpretCmd()
{
	uint8_t value8 = 0;
	uint16_t value16 = 0;

	switch (command[0])
	{
		case 'W':
			//	Walk Mode
			//	Initialize timer 2 to stop movement after a period of 5s.
//			HAL_TIM_Base_Start_IT(&htim2);
//			htim2.Instance->CNT = 0;
			resetTimer(&htim2);
			switch (command[1])
			{
				case 'F':
					//	Move Forwards
					state = WALKING_FORWARD;
					//	Complete
					break;
				case 'B':
					//	Move Bacjwards
					state = WALKING_BACKWARD;
					//	Complete
					break;
				case 'R':
					//	Move Right
					state = WALKING_RIGHT;
					//	Complete
					break;
				case 'L':
					//	Move Left
					state = WALKING_LEFT;
					//	Complete
					break;
				default:
					break;
			}
			break;
		case 'T':
			resetTimer(&htim2);
			switch (command[1])
			{
				case 'R':
					//	Turn Right
					state = TURNING_RIGHT;
					//	Complete
					break;
				case 'L':
					//	Turn Left
					state = TURNING_LEFT;
					//	Complete
					break;
				default:
					break;
			}
			break;
		case 'H':
			switch (command[1])
			{
				case 'O':
					REF = NoRef;
					break;
				case 'A':
					//	Stop or Halt Command
					stop();
					state = STILL;
					break;
				default:
					break;
			}
			break;
		case 'S':
			switch (command[1])
			{
				case 'T':
					state = STAND;
					break;
				case 'I':
					state = SIT;
					break;
				default:
					//	Servo Control Mode
					//	:S#P###
					//	# -> Servo number
					//	### -> Angle
					state = MOVING_JOINT;
					if (command[1])
					{
						value8 = atoi(&command[1]);
					}
					if (command[2] == 'P' && command[3])
					{
						value16 = atoi(&command[3]);
					}
					else if (command[3] == 'P' && command[4])
					{
						value16 = atoi(&command[4]);
					}
					servoControl(anglePos, value8, value16);
					break;
			}
			break;
		case 'L':
			//	Leg Control MOde
			//	:L#*
			// * -> U: Up, D: Down, R: Right, L: Left
			state = MOVING_LEG;
			char movement;
			if (command[1] && command[2] && command[3])
			{
				value8 = atoi(&command[1]);
				movement = command[2];
				value16 = atoi(&command[3]);
				legControl(anglePos, value8, movement, value16);
			}
			break;
		case '?':
			//	Print Position Information
			// Get Position of each leg
			printPosition();
			break;
		case '!':
			// Get State
			getState(state);
			break;
		default:
			break;
	}
}

//	Interruptions
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		RxCounter++;

		uint16_t start = RxBfrPos;
		RxBfrPos = RX_BFR_SIZE - (uint16_t)huart->hdmarx->Instance->CNDTR;
		uint16_t len = RX_BFR_SIZE;

		if (RxRollover < 2)
		{
			if (RxRollover)
			{
				if (RxBfrPos <= start) len = RxBfrPos + RX_BFR_SIZE - start;
				else len = RX_BFR_SIZE + 1;
			}
			else
			{
				len = RxBfrPos - start;
			}
		}
		else
		{
			len = RX_BFR_SIZE + 2;
		}

		int i;
		char data[len];
		memcpy(data, &USART_RxStream[start], len);

		for (i = 0; i < len; i++)
		{
			switch (data[i])
			{
				case ':':
					indRx = 0;
					flagRx = 1;
					break;
				case '\r':
				case ';':
					if (flagRx)
					{
						flagRx = 0;
						command[indRx] = 0;
						InterpretCmd();
					}
					break;
				default:
					if (flagRx)
					{
						command[indRx] = data[i];
						if (indRx < MAX_STRLEN - 1) indRx++;
					}
					break;
			}
		}

		RxRollover = 0;
	}
	else
	{
		RxRollover++;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		if (state != HOME && state != STILL && state != MOVING_JOINT && state != MOVING_LEG)
		{
			stop();
			state = STILL;

		}
	}
	else if (htim == &htim3)
	{
		if (state != STILL && state != HOME)
		{
			HAL_GPIO_TogglePin(START_LED_GPIO_Port, START_LED_Pin);
		}
		else
		{
			HAL_GPIO_WritePin(START_LED_GPIO_Port, START_LED_Pin, GPIO_PIN_RESET);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	stop();
	state = STILL;
	homing(refPos);
	state = HOME;
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
