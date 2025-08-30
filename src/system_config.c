#include "system_config.h"
#include "cmsis_os.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
 void SystemClock_Config(void)
 {
  // RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  // RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // /** Configure the main internal regulator output voltage
  // */
  // __HAL_RCC_PWR_CLK_ENABLE();
  // __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // /** Initializes the RCC Oscillators according to the specified parameters
  // * in the RCC_OscInitTypeDef structure.
  // */
  // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  // RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  // RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  // if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  // /** Initializes the CPU, AHB and APB buses clocks
  // */
  // RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
  //                             |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  // RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  // RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  // RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  // if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  // {
  //   Error_Handler();
  // }

      RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators (PLL on HSI, 84MHz output) */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;     // HSI / 16 = 1 MHz
    RCC_OscInitStruct.PLL.PLLN = 336;    // 1 MHz * 336 = 336 MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // 336 / 4 = 84 MHz SYSCLK
    RCC_OscInitStruct.PLL.PLLQ = 7;      // for USB, can be left as-is

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // HCLK = 84 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    // 42 MHz (max for APB1)
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // 84 MHz (max for APB2)

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
 }
 
 void systemInit(void)
 {
   HAL_Init();
   SystemClock_Config();
   gpio_init();
   propellers_init();
 }

 void gpio_init(void)
 {
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   GPIO_InitTypeDef GPIO_InitStruct_GPIOC = {0};
   __HAL_RCC_GPIOB_CLK_ENABLE();
  //  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);
 
   /*Configure GPIO pin : PA5 */
   GPIO_InitStruct.Pin = LED_PIN1 | LED_PIN2 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct_GPIOC.Pin = BUILTIN_LED;
    GPIO_InitStruct_GPIOC.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_GPIOC.Pull = GPIO_NOPULL;
    GPIO_InitStruct_GPIOC.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_GPIOC);

 }

 void propellers_init()
{
    propellerFR.ID = DRONE_PROPELLER_FRONT_RIGHT; // Set ID for front-right propeller
    propellerFR.Pin = GPIO_PIN_8; // Example pin for front-right propeller
    propellerFR.PropellerTimer = DRONE_PROPELLER_TIMER; // Set the timer for PWM
    propellerFR.TimerChannel = TIM_CHANNEL_1; // Set the timer channel for PWM
    propellerFR.MinDutycyclePeriod = DRONE_PROPELLER_MIN_DUTYCYCLE_PERIOD - 1; // Minimum duty cycle period in microseconds
    propellerFR.MaxDutycyclePeriod = DRONE_PROPELLER_MAX_DUTYCYCLE_PERIOD - 1; // Maximum duty cycle period in microseconds
    propellerFR.CurrentDutycyclePeriode = propellerFR.MinDutycyclePeriod; // Start with minimum duty cycle
    propellerFR.CommunicationProtocol = DRONE_PROPELLER_DSHOT_PROTOCOL; // Set communication protocol to DShot
    drone_propeller_init(&propellerFR); // Initialize front-right propeller

    // propellerFL.ID = DRONE_PROPELLER_FRONT_LEFT; // Set ID for front-left propeller
    // propellerFL.Pin = GPIO_PIN_9; // Example pin for front-left propeller   
    // propellerFL.PropellerTimer = DRONE_PROPELLER_TIMER; // Set the timer for PWM
    // propellerFL.Channel = TIM_CHANNEL_2; // Set the timer channel for PWM
    // propellerFL.MinDutycyclePeriod = 1148; // Minimum duty cycle period in microseconds
    // propellerFL.MaxDutycyclePeriod = 1832; // Maximum duty cycle period in microseconds
    // propellerFL.CurrentDutycyclePeriode = propellerFL.MinDutycyclePeriod; // Start with minimum duty cycle
    // drone_propeller_init(&propellerFL); // Initialize front-left propeller

    // propellerBL.ID = DRONE_PROPELLER_BACK_LEFT; // Set ID for back-left propeller
    // propellerBL.Pin = GPIO_PIN_10; // Example pin for back-left propeller
    // propellerBL.PropellerTimer = DRONE_PROPELLER_TIMER; // Set the timer for PWM
    // propellerBL.Channel = TIM_CHANNEL_3; // Set the timer channel for PWM
    // propellerBL.MinDutycyclePeriod = 1148; // Minimum duty cycle period in microseconds
    // propellerBL.MaxDutycyclePeriod = 1832; // Maximum duty cycle period in microseconds
    // propellerBL.CurrentDutycyclePeriode = propellerBL.MinDutycyclePeriod; // Start with minimum duty cycle
    // drone_propeller_init(&propellerBL); // Initialize back-left propeller

    // propellerBR.ID = DRONE_PROPELLER_BACK_RIGHT; // Set ID for back-right propeller
    // propellerBR.Pin = GPIO_PIN_11; // Example pin for back-right propeller  
    // propellerBR.PropellerTimer = DRONE_PROPELLER_TIMER; // Set the timer for PWM
    // propellerBR.Channel = TIM_CHANNEL_4; // Set the timer channel for PWM
    // propellerBR.MinDutycyclePeriod = 1148; // Minimum duty cycle period in microseconds
    // propellerBR.MaxDutycyclePeriod = 1832; // Maximum duty cycle period in microseconds
    // propellerBR.CurrentDutycyclePeriode = propellerBR.MinDutycyclePeriod; // Start with minimum duty cycle
    // drone_propeller_init(&propellerBR); // Initialize back-right propeller

    // Start the propellers
    if (drone_propeller_start() != DRONE_ERROR_NONE) {
        while (1)
        {
          // Wait indefinitely
        }
    }

    // Set initial speed for each propeller   
    // drone_propeller_set_speed(DRONE_PROPELLER_FRONT_RIGHT, 100); // Set speed to 0% for front-right propeller 
    // HAL_Delay(6500); // Delay for 5 seconds
    HAL_GPIO_WritePin(GPIOB, LED_PIN2, GPIO_PIN_SET); // Turn on the built-in LED
    drone_propeller_set_speed(DRONE_PROPELLER_FRONT_RIGHT, 0); // Set speed to 0% for front-right propeller
    HAL_Delay(6000); // Delay for 5 seconds
    // drone_propeller_set_speed(DRONE_PROPELLER_FRONT_RIGHT, 50);
    // drone_propeller_set_speed(DRONE_PROPELLER_FRONT_LEFT, 100); // Set speed to 0% for front-left propeller
    // HAL_Delay(9000); // Delay for 5 seconds
    // drone_propeller_set_speed(DRONE_PROPELLER_FRONT_LEFT, 0); // Set speed to 0% for front-left propeller
    // drone_propeller_set_speed(DRONE_PROPELLER_BACK_LEFT, 0); // Set speed to 0% for back-left propeller
    // drone_propeller_set_speed(DRONE_PROPELLER_BACK_RIGHT, 0); // Set speed to 0% for back-right propeller
}