/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* ETH_CODE: add lwiperf, see comment in StartDefaultTask function */
#include "lwip/apps/lwiperf.h"
#include "shared_state.h"
#include "watchdog.h"
#include "telemetry.h"
#include "esp_link.h"
#include "idlog_tx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

IWDG_HandleTypeDef hiwdg2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* Watchdog task: gates IWDG2 + WWDG2 refresh on M7 heartbeat.
 * Stack 256 words (1024 B) per checklist; priority AboveNormal so it
 * preempts LWIP / telemetry / lwiperf and refreshes on time. */
static osThreadId_t        watchdogTaskHandle;
static const osThreadAttr_t watchdogTask_attributes = {
    .name       = "watchdogTask",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t) osPriorityAboveNormal,
};

/* Telemetry task: UDP TX to MATLAB at 100 Hz (Step 6 — body filled later).
 * Stack 512 words per checklist; Normal priority. */
static osThreadId_t        telemetryTaskHandle;
static const osThreadAttr_t telemetryTask_attributes = {
    .name       = "telemetryTask",
    .stack_size = 512 * 4,
    .priority   = (osPriority_t) osPriorityNormal,
};

/* ESP32 UART link tasks (§8). RX runs slightly hotter than TX so incoming
 * commands are applied to g_shared with low latency. */
static osThreadId_t        espRxTaskHandle;
static const osThreadAttr_t espRxTask_attributes = {
    .name       = "espRxTask",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t) osPriorityAboveNormal,
};

static osThreadId_t        espTxTaskHandle;
static const osThreadAttr_t espTxTask_attributes = {
    .name       = "espTxTask",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t) osPriorityNormal,
};

/* High-rate ID/balance log streamer: drains g_shared.idlog[] to UDP 5007
 * every 10 ms during SYSID/BALANCE runs (idlog_tx.c). Normal priority —
 * losing a drain slot only delays samples, the 128-sample ring absorbs it. */
static osThreadId_t        idLogTxTaskHandle;
static const osThreadAttr_t idLogTxTask_attributes = {
    .name       = "idLogTxTask",
    .stack_size = 512 * 4,
    .priority   = (osPriority_t) osPriorityNormal,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_IWDG2_Init(void);
static void MX_WWDG2_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
/* StartWatchdogTask  lives in watchdog.c (Step 7) */
/* StartTelemetryTask lives in telemetry.c (Step 6) */
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

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/* ETH_CODE: fixed core synchronization
	 * Busy wait, since entering STOP mode breaks debug session.
	 *
	 * HANDOFF §5.2: M7 releases HSEM_ID_0 exactly once per M7 boot. If M4
	 * resets alone (WWDG2, reflash, debugger reset) the flag never comes and
	 * the bare spin hangs forever. Fallback: g_shared lives in D2 SRAM which
	 * survives an M4-only warm reset, so magic + m7_ready still reading valid
	 * means M7 is up and the shared region is initialised — proceed. */
	__HAL_RCC_HSEM_CLK_ENABLE();
	__HAL_RCC_D2SRAM3_CLK_ENABLE();   /* g_shared lives in D2 SRAM3 */
	/* Latch releases into C2ISR: without notification activation the ISR
	 * flag never sets for this core and the poll below could only ever exit
	 * through the g_shared fallback (ST's reference busy-wait does this too). */
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
	while(((__HAL_HSEM_GET_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0))) == 0)
	      && !(g_shared.magic == SHARED_STATE_MAGIC && g_shared.m7_ready == 1u));
	__HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
	HAL_HSEM_DeactivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
#ifdef DEBUG
  /* Freeze the D2 watchdogs while the CM4 core is halted by a debugger.
   * Without these bits, a breakpoint / single-step / flash-loader halt lets
   * IWDG2+WWDG2 keep counting and the chip resets ~20 ms into every halt —
   * which also aborts flash erases (the "need BOOT0 jumper to flash" trap).
   * The bits only act while a debugger has the core halted, but guard them
   * to DEBUG builds anyway so production silicon never touches DBGMCU. */
  __HAL_DBGMCU_FREEZE2_IWDG2();
  __HAL_DBGMCU_FREEZE2_WWDG2();
#endif
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_IWDG2_Init();
  MX_WWDG2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* M7 has released HSEM_ID_BOOT_SYNC by now (caught above). Wait for the
   * shared D2 region to be fully initialised (m7_ready=1) before we run
   * any code that touches g_shared.
   *
   * Reality check (arch review 2026-07-11): WWDG2 started counting in
   * MX_WWDG2_Init just above and nothing refreshes it until the watchdog
   * task's first slot — ~21 ms of budget (PCLK1 100 MHz / 4096 / 8,
   * 64 counts to reset). So the 2 s timeout below can never actually
   * elapse: if m7_ready were late, WWDG2 resets the chip first and the
   * failure presents as a REBOOT LOOP, not a halt in Error_Handler.
   * (Normal boots spend ~5-15 ms here; the fallback in the spin above
   * means m7_ready is virtually always already set on arrival.) Proper
   * boot-failure visibility is fix-plan step 13 (batch C). */
  if (!SharedState_M4_WaitReady(2000u /* ms */)) {
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  watchdogTaskHandle  = osThreadNew(StartWatchdogTask,  NULL, &watchdogTask_attributes);
  telemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &telemetryTask_attributes);
  espRxTaskHandle     = osThreadNew(EspLink_RxTask,     NULL, &espRxTask_attributes);
  espTxTaskHandle     = osThreadNew(EspLink_TxTask,     NULL, &espTxTask_attributes);
  idLogTxTaskHandle   = osThreadNew(StartIdLogTxTask,   NULL, &idLogTxTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 8;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 8;
  PeriphClkInitStruct.PLL2.PLL2Q = 8;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG2_Init(void)
{

  /* USER CODE BEGIN IWDG2_Init 0 */

  /* USER CODE END IWDG2_Init 0 */

  /* USER CODE BEGIN IWDG2_Init 1 */

  /* USER CODE END IWDG2_Init 1 */
  hiwdg2.Instance = IWDG2;
  hiwdg2.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg2.Init.Window = 4095;
  hiwdg2.Init.Reload = 2000;
  if (HAL_IWDG_Init(&hiwdg2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG2_Init 2 */

  /* USER CODE END IWDG2_Init 2 */

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
  huart2.Init.BaudRate = 921600;
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
  * @brief WWDG2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG2_Init(void)
{

  /* USER CODE BEGIN WWDG2_Init 0 */

  /* USER CODE END WWDG2_Init 0 */

  /* USER CODE BEGIN WWDG2_Init 1 */
  /* CubeMX didn't generate NVIC entry for WWDG_IRQn even though EWI is on.
   * Set the priority and enable it here, before HAL_WWDG_Init so that
   * HAL_WWDG_MspInit doesn't conflict (it leaves NVIC alone for WWDG). */
  HAL_NVIC_SetPriority(WWDG_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(WWDG_IRQn);
  /* USER CODE END WWDG2_Init 1 */
  hwwdg2.Instance = WWDG2;
  hwwdg2.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg2.Init.Window = 0x7F;
  hwwdg2.Init.Counter = 0x7F;
  hwwdg2.Init.EWIMode = WWDG_EWI_ENABLE;
  if (HAL_WWDG_Init(&hwwdg2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG2_Init 2 */

  /* USER CODE END WWDG2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* StartWatchdogTask  is implemented in watchdog.c (Step 7) */
/* StartTelemetryTask is implemented in telemetry.c (Step 6) */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
#ifdef ENABLE_LWIPERF
	/* ETH_CODE: lwiperf TCP throughput test harness (iperf 2.0.6 on the
	 * host; iperf3 incompatible). Leftover from the ETH bring-up example —
	 * now OPT-IN (arch review F15a, 2026-07-11): the default build must not
	 * start a TCP server AND a client toward the MATLAB host on every boot.
	 * An accidental iperf session contends for pbuf pool and CPU exactly
	 * during high-rate telemetry/ID-log windows. Define ENABLE_LWIPERF in
	 * the CM4 build settings for a dedicated network-throughput session. */
    LOCK_TCPIP_CORE();
	lwiperf_start_tcp_server_default(NULL, NULL);

	ip4_addr_t remote_addr;
	IP4_ADDR(&remote_addr, 192, 168, 1, 5);
	lwiperf_start_tcp_client_default(&remote_addr, NULL, NULL);
	UNLOCK_TCPIP_CORE();
#endif /* ENABLE_LWIPERF */
	/* Infinite loop */
	for(;;)
	{
	  osDelay(1000);
	}
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
