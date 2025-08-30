#include "drone_propeller.h"
#include "math.h"

void drone_propeller_init(DronePropeller_t *propeller)
{
    propeller->PropellerTimer = DRONE_PROPELLER_TIMER; // Set the timer for PWM
    if (propeller->CommunicationProtocol == DRONE_PROPELLER_PWM_PROTOCOL) {
        propeller->PWMPeriod = 1000000 / DRONE_PROPELLER_PWM_FREQ; // Calculate PWM period in microseconds
    } else if (propeller->CommunicationProtocol == DRONE_PROPELLER_DSHOT_PROTOCOL) {
        propeller->DshotPeriod = 1000000.0 / DRONE_PROPELLER_DSHOT600_FREQ; // Calculate DShot period in micseconds
        float system_period = 1000000.0 / SYSTEMCLOCK_FREQUENCY; // System period in microseconds
        propeller->DshotTickPeriod = round(propeller->DshotPeriod / system_period) - 1; //period in timer ticks
        propeller->DshotHighBitTick = round(propeller->DshotTickPeriod * (DRONE_PROPELLER_DSHOT_HIGH_BIT_DUTYCYCLE / 100.0)); // High bit tick in microseconds
        propeller->DshotLowBitTick = round(propeller->DshotTickPeriod * (DRONE_PROPELLER_DSHOT_LOW_BIT_DUTYCYCLE / 100.0)); // Low bit tick in microseconds
    }

    if (drone_propeller_length >= 4) {
        // Handle error: maximum number of propellers reached
        return; // or handle error appropriately
    }   
    drone_propeller_array[drone_propeller_length] = propeller; // Add to the array
    drone_propeller_length++; // Increment the count of propellerse
}

void drone_propeller_send_dshot(DronePropellerChannel_t channel, uint32_t period)
{
    // Start the DMA transfer to send the DShot command
    if (drone_propeller_dma_is_busy[channel]) {
        // Handle error: DMA is busy
        return; // or handle error appropriately
    }

    DronePropeller_t *propeller = drone_propeller_array[channel]; // Get the propeller based on the channel
    if (propeller == NULL) {
        // Handle error: propeller not found
        return; // or handle error appropriately
    }

    uint16_t dshot_command = 0x0000; // Initialize DShot command
    uint16_t dshot_speed = ((uint16_t) period) & 0x07FF; // Convert speed to DShot format and ensure it fits in 11 bits
    uint16_t dshot_telemetry = 0;
    dshot_telemetry &= 0x0000; // Set telemetry bit (0 for no telemetry, 1 for telemetry)
    
    dshot_command |= (dshot_speed << 5); // Shift the speed to the correct position in the command
    dshot_command |= (dshot_telemetry << 4); // Set telemetry bit

    uint8_t crc = 0; // Initialize CRC
    for (uint8_t i = 1; i <= 3; i++) {
        crc ^= (dshot_command >> (4 * i)) & 0x0F; // XOR the upper 4 bits
    }

    crc = crc & 0x000F; // Ensure CRC fits in 4 bits

    dshot_command |= crc; // Set the CRC in the command

    // Fill the DMA buffer with the DShot command
    for (int i = 15; i >= 0; i--) {
        uint8_t bit = (dshot_command >> i) & 0x0001; // Get the bit to send
        uint8_t index = 15 - i;
        if (bit) {
            drone_propeller_dma_buffer[channel][index] = propeller->DshotHighBitTick; // Set high period
        } else {
            drone_propeller_dma_buffer[channel][index] = propeller->DshotLowBitTick; // Set low period
        }
    }
    drone_propeller_dma_is_busy[channel] = true; // Set DMA busy flag
    HAL_TIM_PWM_Start_DMA(&drone_propeller_timer, propeller->TimerChannel, (uint32_t *)drone_propeller_dma_buffer[channel], 16);
}

void drone_propeller_set_speed(DronePropellerChannel_t channel, uint32_t speed)
{
    // Ensure speed is within the defined range
    if (speed < DRONE_PROPELLER_MIN_SPEED || speed > DRONE_PROPELLER_MAX_SPEED) {
        return; // or handle error appropriately
    }

    DronePropeller_t *propeller = drone_propeller_array[channel]; // Get the propeller based on the channel
    if (propeller == NULL) {
        // Handle error: propeller not found
        return; // or handle error appropriately
    }

    uint32_t speed_to_period = (uint32_t)((((speed * (propeller->MaxDutycyclePeriod - propeller->MinDutycyclePeriod)) / (float)(DRONE_PROPELLER_MAX_SPEED)) + propeller->MinDutycyclePeriod));
    if (speed_to_period < propeller->MinDutycyclePeriod) {
        speed_to_period = propeller->MinDutycyclePeriod; // Ensure it does not go below minimum
    } else if (speed_to_period > propeller->MaxDutycyclePeriod) {
        speed_to_period = propeller->MaxDutycyclePeriod; // Ensure it does not exceed maximum
    }

    // Calculate the duty cycle period based on the speed
    if (propeller->CommunicationProtocol == DRONE_PROPELLER_PWM_PROTOCOL) {
        // For PWM protocol, set the PWM duty cycle
        propeller->CurrentDutycyclePeriode = speed_to_period; // Update current duty cycle period
        __HAL_TIM_SET_COMPARE(&drone_propeller_timer, propeller->TimerChannel, speed_to_period); // Set the PWM duty cycle
    } else if (propeller->CommunicationProtocol == DRONE_PROPELLER_DSHOT_PROTOCOL) {
        // For DShot protocol, handle the DShot communication
        // DShot implementation would go here, typically involving DMA transfer
        drone_propeller_send_dshot(channel, speed_to_period); // Call the DShot send function
    }
}

DroneError_t drone_propeller_start()
{
    //check if the propeller array communication protocol is not same
    for (uint16_t i = 1; i < drone_propeller_length; i++) {
        if (drone_propeller_array[i]->CommunicationProtocol != drone_propeller_array[0]->CommunicationProtocol) {
            // Handle error: communication protocols do not match
            return DRONE_ERROR_INVALID_PARAM;
        }
    }
    __HAL_RCC_TIM1_CLK_ENABLE(); // Enable clock for TIM1
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock for GPIOA
    // __HAL_TIM_MOE_ENABLE(&drone_propeller_timer);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Initialize GPIO for each propeller
    uint32_t pins = 0;
    for (uint16_t i = 0; i < drone_propeller_length; i++) {
        DronePropeller_t *propeller = drone_propeller_array[i];
        pins |= propeller->Pin; // Combine all pins into a single variable
    }
    
    // Configure GPIO pins for output
    GPIO_InitStruct.Pin = pins;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate function push-pull
    GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // Set speed to low
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1; // Set alternate function for TIM1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Initialize GPIOA with the configured settings

    // Initialize the timer for PWM
    if (drone_propeller_array[0]->CommunicationProtocol == DRONE_PROPELLER_PWM_PROTOCOL) {
        drone_propeller_timer.Instance = DRONE_PROPELLER_TIMER;
        drone_propeller_timer.Init.Prescaler = 83; // Set frequency to 1 MHz (assuming 84 MHz clock)
        drone_propeller_timer.Init.CounterMode = TIM_COUNTERMODE_UP; // Count up
        // Set the period based on the PWM frequency
        drone_propeller_timer.Init.Period = drone_propeller_array[0]->PWMPeriod - 1; // Set period for PWM
        drone_propeller_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // No clock division
        drone_propeller_timer.Init.RepetitionCounter = 0; // No repetition
        drone_propeller_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Disable auto-reload preload
        if (HAL_TIM_Base_Init(&drone_propeller_timer) != HAL_OK) {
            // Handle error: timer initialization failed
            return DRONE_ERROR_SYSTEM_INITIALIZATION;
        }

        TIM_OC_InitTypeDef sConfigOC = {0};
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = 0;                      // Start with 0% duty cycle
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

        for (uint16_t i = 0; i < drone_propeller_length; i++) {
            DronePropeller_t *propeller = drone_propeller_array[i];
            sConfigOC.Pulse = propeller->MinDutycyclePeriod;
            if (HAL_TIM_PWM_ConfigChannel(&drone_propeller_timer, &sConfigOC, propeller->TimerChannel) != HAL_OK) {
                // Handle error: PWM configuration failed
                return DRONE_ERROR_SYSTEM_INITIALIZATION;
            }
            HAL_TIM_PWM_Start(&drone_propeller_timer, propeller->TimerChannel); // Start PWM on the Channel
            // __HAL_TIM_SET_COMPARE(&drone_propeller_timer, propeller->TimerChannel, propeller->MinDutycyclePeriod); // Set initial duty cycle
        }
    }else if (drone_propeller_array[0]->CommunicationProtocol == DRONE_PROPELLER_DSHOT_PROTOCOL) {
        __HAL_RCC_DMA2_CLK_ENABLE(); // Enable clock for DMA2
        // Initialize the timer for DShot
        drone_propeller_timer.Instance = DRONE_PROPELLER_TIMER;
        drone_propeller_timer.Init.Prescaler = 0; // Set prescaler to 0 for maximum frequency
        drone_propeller_timer.Init.CounterMode = TIM_COUNTERMODE_UP; // Count up
        drone_propeller_timer.Init.Period = drone_propeller_array[0]->DshotTickPeriod; // Set period for DShot
        drone_propeller_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // No clock division
        drone_propeller_timer.Init.RepetitionCounter = 0; // No repetition
        drone_propeller_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Disable auto-reload preload
        if (HAL_TIM_Base_Init(&drone_propeller_timer) != HAL_OK) {
            // Handle error: timer initialization failed
            return DRONE_ERROR_SYSTEM_INITIALIZATION;
        }

        for (uint16_t i = 0; i < drone_propeller_length; i++) {
            switch (i)
            {
            case 0:
                drone_propeller_dma[i].Instance = DRONE_PROPELLER_DMA_TIM1_CHANNEL1;
                HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0); 
                HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
                break;
            case 1:
                drone_propeller_dma[i].Instance = DRONE_PROPELLER_DMA_TIM1_CHANNEL2;
                HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0); 
                HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
                break;
            case 2:
                drone_propeller_dma[i].Instance = DRONE_PROPELLER_DMA_TIM1_CHANNEL3;
                HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
                break;
            case 3:
                drone_propeller_dma[i].Instance = DRONE_PROPELLER_DMA_TIM1_CHANNEL4;
                HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
                break;
            default:
                break;
            }
            drone_propeller_dma[i].Init.Channel = DMA_CHANNEL_6; // Set the DMA channel for TIM1
            drone_propeller_dma[i].Init.Direction = DMA_MEMORY_TO_PERIPH; // Memory to peripheral direction
            drone_propeller_dma[i].Init.PeriphInc = DMA_PINC_DISABLE; // Peripheral address not incremented
            drone_propeller_dma[i].Init.MemInc = DMA_MINC_ENABLE; // Memory address incremented
            drone_propeller_dma[i].Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // Peripheral data alignment
            drone_propeller_dma[i].Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD; // Memory data alignment
            drone_propeller_dma[i].Init.Mode = DMA_NORMAL; // Normal mode
            drone_propeller_dma[i].Init.Priority = DMA_PRIORITY_HIGH; // High priority
            drone_propeller_dma[i].Init.FIFOMode = DMA_FIFOMODE_DISABLE; // FIFO mode disabled

            HAL_DMA_Init(&drone_propeller_dma[i]); // Initialize DMA

            //configure NVIC for DMA
            TIM_OC_InitTypeDef sConfigOC = {0};
            sConfigOC.OCMode = TIM_OCMODE_PWM1;
            sConfigOC.Pulse = 0;                      // Start with 0% duty cycle
            sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
            sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
            DronePropeller_t *propeller = drone_propeller_array[i];
            if (HAL_TIM_PWM_ConfigChannel(&drone_propeller_timer, &sConfigOC, propeller->TimerChannel) != HAL_OK) {
                // Handle error: PWM configuration failed
                return DRONE_ERROR_SYSTEM_INITIALIZATION;
            }
            __HAL_LINKDMA(&drone_propeller_timer, hdma[i], drone_propeller_dma[i]); // Link DMA to the timer
            // HAL_TIM_PWM_Start_DMA(&drone_propeller_timer, propeller->TimerChannel, (uint32_t *)&drone_propeller_dma_buffer[i], 16); // Start PWM on the Channel with DMA
        }
    }else {
        // Handle error: unsupported communication protocol
        return DRONE_ERROR_SYSTEM_INITIALIZATION;
    }
}

void DMA2_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&drone_propeller_dma[DRONE_PROPELLER_FRONT_RIGHT]);
}

void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&drone_propeller_dma[DRONE_PROPELLER_FRONT_LEFT]);
}

void DMA2_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&drone_propeller_dma[DRONE_PROPELLER_BACK_LEFT]);
}

void DMA2_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&drone_propeller_dma[DRONE_PROPELLER_BACK_RIGHT]);
}

void HAL_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma == &drone_propeller_dma[DRONE_PROPELLER_FRONT_RIGHT])
    {
        drone_propeller_dma_is_busy[DRONE_PROPELLER_FRONT_RIGHT] = false;
    }
    else if (hdma == &drone_propeller_dma[DRONE_PROPELLER_FRONT_LEFT])
    {
        drone_propeller_dma_is_busy[DRONE_PROPELLER_FRONT_LEFT] = false;
    }
    else if (hdma == &drone_propeller_dma[DRONE_PROPELLER_BACK_LEFT])
    {
        drone_propeller_dma_is_busy[DRONE_PROPELLER_BACK_LEFT] = false;
    }
    else if (hdma == &drone_propeller_dma[DRONE_PROPELLER_BACK_RIGHT])
    {
        drone_propeller_dma_is_busy[DRONE_PROPELLER_BACK_RIGHT] = false;
    }
}
