/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MovingAverage.h"
#include "ConvertValues.h"
#include "IgnitionMap.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* ---------------------------------------------------------------------------*/
/*             Variables for camshaft rpm sensor and pressure                 */
/* ---------------------------------------------------------------------------*/
extern volatile uint16_t mapSensorValue;
extern volatile uint8_t adc_ready;

volatile uint32_t rising_ticks = 0;
volatile uint32_t diff = 0;

volatile uint8_t rising_valid = 0;
volatile uint8_t overflows = 0;

volatile uint8_t ignore_counter = 0;

/* ---------------------------------------------------------------------------*/
/*                           Ignition variables                               */
/* ---------------------------------------------------------------------------*/

volatile uint8_t last_cylinder = 0;

/** Timer variables **/
TIM_OC_InitTypeDef sConfigOC_TIM4_CH1 = {0};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1, ADC2 and ADC3 interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
	mapSensorValue = HAL_ADC_GetValue(&hadc2);
	adc_ready = 1;

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

	/*
	 * Check if we are called due to an overflow
	 */
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET)
		{
			/* Only count the overflow if it is between rising and falling edge */
			if(rising_valid)
			{
				++overflows;
			}

			__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
			__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

			goto end_timer3_interrupt;
		}
	}


	/*
	 *  Capture compare 1 event (Channel 1 = Camshaft Sensor Rising Edge)
	 */
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC1) != RESET)
		{
			{
				__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
				htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_1;

				/* Input capture event */
				if ((htim3.Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U)
				{
					rising_ticks = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
					if(ignore_counter >= 20)
					{
						rising_valid = 1;
					}
					goto end_timer3_interrupt;
				}
			}
		}
	}


	/*
	 * Capture compare 2 event (Channel 2 = Camshaft Sensor Falling Edge)
	 */
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC2) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC2) != RESET)
		{
			__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
			htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_2;

			/* Input capture event */
			if ((htim3.Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00U)
			{

				/* Ignore the first 30 ticks after startup */
				if(ignore_counter < 30)
				{
					++ignore_counter;
					goto end_timer3_interrupt;
				}

				/* Only continue if we have a rising edge value from before */
				if(!rising_valid)
				{
					goto end_timer3_interrupt;
				}

				/* Read new timer ticks of this interrupt (falling edge) */
				uint32_t falling_ticks = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);

				/* Calculate diff between rising and falling (x2 to get T_period) */
				if(overflows > 0)
				{
					/*
					 * Rising edge is valid and overflow occurred
					 * --> Overflow was between rising and falling edge
					 */

					diff = (uint32_t)65535 - rising_ticks + falling_ticks;
					overflows = 0;
				}
				else
				{
					diff = falling_ticks - rising_ticks;
				}
				diff *= 2;

				/* Fail-safe (just in case something is wrong and diff overflows) */
				if(diff > 65535)
				{
					rising_valid = 0;
					goto end_timer3_interrupt;
				}

				/* Check if we reached OT */
				float curAvg = (float)(MovAvg_getMean());
				float left = (float)curAvg*1.6f;
				float right = (float)curAvg*2.4f;

				/* OT reached? */
				if((diff > 0) && (diff > left) && (diff < right))
				{
					int firingAngle = IgnitionMap_getFiringAngle(mapSensorValue, calculateEngineRPM(curAvg));

					/* Calculate timer ticks for ignitions */
					float ticks_per_round = curAvg * 38.0f;
					float ignition_ticks = ticks_per_round / 4.0f;
					float ticks_per_degree = ignition_ticks / 90.0f;
					float ticks_before_ot = ((float)firingAngle/10.0f) * ticks_per_degree + 5; /* 5 additional ticks to compensate for delay */

					/*
					 * Set ticks for Timer 4 to start timer for ignition 2.
					 */
					last_cylinder = 0;

					__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
					__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
					htim4.Instance->ARR = (uint16_t)(round(ignition_ticks));
					htim4.Instance->CNT = (uint16_t)(round(ticks_before_ot));
					__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

					/* Enable generation of update event */
					__HAL_TIM_URS_ENABLE(&htim4);
					__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
					__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
				}
				else
				{
					MovAvg_update(diff);
				}

				rising_valid = 0;
			}
		}
	}

	end_timer3_interrupt:
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

	/* TIM4 Update event = Overflow --> Ignition x */
	if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET)
		{
			/* Ignite correct pin */

			switch(last_cylinder)
			{
			case 0: /* Now ignite cylinder 4 */
				HAL_GPIO_WritePin(Zylinder24_Zuendung_GPIO_Port, Zylinder24_Zuendung_Pin, GPIO_PIN_RESET);
				last_cylinder = 1;

				/* Set compare for Compare Channel 1 (sets pin back to low after 5 ticks) */
				htim4.Instance->CCER &= ~TIM_CCER_CC1E; /* Disable channel 1 */
				htim4.Instance->CCR1 = (uint16_t)(htim4.Instance->CNT) + 20; /* Update period */
				htim4.Instance->CCER |= TIM_CCER_CC1E; /* Enable channel 1 */

				__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);

				break;

			case 1: /* Now ignite cylinder 3 */
				HAL_GPIO_WritePin(Zylinder13_Zuendung_GPIO_Port, Zylinder13_Zuendung_Pin, GPIO_PIN_RESET);
				last_cylinder = 2;

				/* Set compare for Compare Channel 1 (sets pin back to low after 5 ticks) */
				htim4.Instance->CCER &= ~TIM_CCER_CC1E; /* Disable channel 1 */
				htim4.Instance->CCR1 = (uint16_t)(htim4.Instance->CNT) + 20; /* Update period */
				htim4.Instance->CCER |= TIM_CCER_CC1E; /* Enable channel 1 */

				break;

			case 2: /* Now ignite cylinder 2 */
				HAL_GPIO_WritePin(Zylinder24_Zuendung_GPIO_Port, Zylinder24_Zuendung_Pin, GPIO_PIN_RESET);
				last_cylinder = 3;

				/* Set compare for Compare Channel 1 (sets pin back to low after 5 ticks) */
				htim4.Instance->CCER &= ~TIM_CCER_CC1E; /* Disable channel 1 */
				htim4.Instance->CCR1 = (uint16_t)(htim4.Instance->CNT) + 20; /* Update period */
				htim4.Instance->CCER |= TIM_CCER_CC1E; /* Enable channel 1 */

				break;

			case 3: /* Now ignite cylinder 1 */
				HAL_GPIO_WritePin(Zylinder13_Zuendung_GPIO_Port, Zylinder13_Zuendung_Pin, GPIO_PIN_RESET);
				last_cylinder = 0;

				/* Set compare for Compare Channel 1 (sets pin back to low after 5 ticks) */
				htim4.Instance->CCER &= ~TIM_CCER_CC1E; /* Disable channel 1 */
				htim4.Instance->CCR1 = (uint16_t)(htim4.Instance->CNT) + 20; /* Update period */
				htim4.Instance->CCER |= TIM_CCER_CC1E; /* Enable channel 1 */


				/* Disable update flag to avoid another ignition until next OT enables TIM4 update event again */
				__HAL_TIM_URS_DISABLE(&htim4);
				__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
				break;

				/* Wrong value: Rather do nothing instead of doing something wrong */
			default:
				__HAL_TIM_URS_DISABLE(&htim4);
				__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
				break;
			}

			/* Reset update flag */
			__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
			__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

			/* Done with interrupt */
			goto end_timer4;
		}
	}


	/* TIM4 Compare event = Set pin x back to low */
	if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC1) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_CC1) != RESET)
		{
			__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC1);

			/* Output compare event */
			if(!((htim4.Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U))
			{
				switch(last_cylinder)
				{
				case 1: /* Cylinder 4 was ignited before */
					HAL_GPIO_WritePin(Zylinder24_Zuendung_GPIO_Port, Zylinder24_Zuendung_Pin, GPIO_PIN_SET);

//					HAL_ADC_Start_IT(&hadc2);

					break;

				case 2: /* Cylinder 3 was ignited before */
					HAL_GPIO_WritePin(Zylinder13_Zuendung_GPIO_Port, Zylinder13_Zuendung_Pin, GPIO_PIN_SET);
					break;

				case 3: /* Cylinder 2 was ignited before */
					HAL_GPIO_WritePin(Zylinder24_Zuendung_GPIO_Port, Zylinder24_Zuendung_Pin, GPIO_PIN_SET);
					break;

				case 0: /* Cylinder 1 was ignited before */
					HAL_GPIO_WritePin(Zylinder13_Zuendung_GPIO_Port, Zylinder13_Zuendung_Pin, GPIO_PIN_SET);
					break;

					/* Wrong value: Rather set both pins to low instead of doing nothing */
				default:
					HAL_GPIO_WritePin(Zylinder13_Zuendung_GPIO_Port, Zylinder13_Zuendung_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(Zylinder24_Zuendung_GPIO_Port, Zylinder24_Zuendung_Pin, GPIO_PIN_SET);
					break;
				}
			}
		}
	}


end_timer4:
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void init_TIM4_stuff() {
	sConfigOC_TIM4_CH1.OCMode = TIM_OCMODE_TIMING;
	sConfigOC_TIM4_CH1.Pulse = 0;
	sConfigOC_TIM4_CH1.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC_TIM4_CH1.OCFastMode = TIM_OCFAST_DISABLE;
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
