#include "drone_spi_communication.h"

SPI_HandleTypeDef drone_spi;
DMA_HandleTypeDef drone_spi_tx_dma;
DMA_HandleTypeDef drone_spi_rx_dma;

/* DMA transfer buffers (must be persistent, not on stack) */
uint8_t spi_tx_buffer[32];
uint8_t spi_rx_buffer[32];
volatile bool spi_txrx_complete = false;
volatile bool spi_tx_complete = false;
volatile DroneSPISensorType_t current_spi_sensor = DRONE_SPI_NO_SENSOR; // Default to no sensor

void drone_spi_init(void)
{
    /* Enable clocks */
    DRONE_ENABLE_SPI_CLK();
    DRONE_ENABLE_SPI_GPIO_CLK();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* SCK, MISO, MOSI — alternate function */
    GPIO_InitStruct.Pin = DRONE_SCK_PIN | DRONE_MISO_PIN | DRONE_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(DRONE_SPI_GPIO, &GPIO_InitStruct);

    /* Initialize DMA before SPI */
    drone_spi_dma_init();

    /* SPI peripheral config */
    drone_spi.Instance = DRONE_SPI_INSTANCE;
    drone_spi.Init.Mode = SPI_MODE_MASTER;
    drone_spi.Init.Direction = SPI_DIRECTION_2LINES;
    drone_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    drone_spi.Init.CLKPolarity = SPI_POLARITY_HIGH;               // CPOL=1 (Mode 3)
    drone_spi.Init.CLKPhase = SPI_PHASE_2EDGE;                    // CPHA=1 (Mode 3)
    drone_spi.Init.NSS = SPI_NSS_SOFT;                            // CS managed manually
    drone_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 84MHz/128 = ~656KHz (<1MHz limit)
    drone_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    drone_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    drone_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    if (HAL_SPI_Init(&drone_spi) != HAL_OK)
    {
        Error_Handler();
    }

    /* Link DMA handles to SPI handle */
    __HAL_LINKDMA(&drone_spi, hdmatx, drone_spi_tx_dma);
    __HAL_LINKDMA(&drone_spi, hdmarx, drone_spi_rx_dma);
}

void drone_spi_dma_init(void)
{
    /* Enable DMA2 clock */
    DRONE_ENABLE_DMA_CLK();

    /* Configure DMA for SPI1 TX (DMA2 Stream3 Channel3) */
    drone_spi_tx_dma.Instance = DRONE_SPI_TX_DMA_STREAM;
    drone_spi_tx_dma.Init.Channel = DRONE_SPI_DMA_CHANNEL;
    drone_spi_tx_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    drone_spi_tx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    drone_spi_tx_dma.Init.MemInc = DMA_MINC_ENABLE;
    drone_spi_tx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    drone_spi_tx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    drone_spi_tx_dma.Init.Mode = DMA_NORMAL;
    drone_spi_tx_dma.Init.Priority = DMA_PRIORITY_HIGH;
    drone_spi_tx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&drone_spi_tx_dma) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure DMA for SPI1 RX (DMA2 Stream2 Channel3) */
    drone_spi_rx_dma.Instance = DRONE_SPI_RX_DMA_STREAM;
    drone_spi_rx_dma.Init.Channel = DRONE_SPI_DMA_CHANNEL;
    drone_spi_rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    drone_spi_rx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    drone_spi_rx_dma.Init.MemInc = DMA_MINC_ENABLE;
    drone_spi_rx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    drone_spi_rx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    drone_spi_rx_dma.Init.Mode = DMA_NORMAL;
    drone_spi_rx_dma.Init.Priority = DMA_PRIORITY_HIGH;
    drone_spi_rx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&drone_spi_rx_dma) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure NVIC for DMA interrupts - DISABLED by default for init */
    // Interrupts will be enabled later by calling drone_spi_enable_dma_interrupts()
    // HAL_NVIC_SetPriority(DRONE_SPI_TX_DMA_IRQn, 5, 0);
    // HAL_NVIC_EnableIRQ(DRONE_SPI_TX_DMA_IRQn);

    // HAL_NVIC_SetPriority(DRONE_SPI_RX_DMA_IRQn, 5, 0);
    // HAL_NVIC_EnableIRQ(DRONE_SPI_RX_DMA_IRQn);
}

bool is_spi_initialized(void)
{
    return (drone_spi.State == HAL_SPI_STATE_READY);
}

/* Enable DMA interrupts - call before starting RTOS */
void drone_spi_enable_dma_interrupts(void)
{
    HAL_NVIC_SetPriority(DRONE_SPI_TX_DMA_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DRONE_SPI_TX_DMA_IRQn);

    HAL_NVIC_SetPriority(DRONE_SPI_RX_DMA_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DRONE_SPI_RX_DMA_IRQn);
}

/* Disable DMA interrupts */
void drone_spi_disable_dma_interrupts(void)
{
    HAL_NVIC_DisableIRQ(DRONE_SPI_TX_DMA_IRQn);
    HAL_NVIC_DisableIRQ(DRONE_SPI_RX_DMA_IRQn);
}

/* Synchronous (blocking) SPI functions - for initialization */
HAL_StatusTypeDef drone_spi_transmit_blocking(uint8_t *pData, uint16_t Size, uint32_t timeout_ms)
{
    return HAL_SPI_Transmit(&drone_spi, pData, Size, timeout_ms);
}

HAL_StatusTypeDef drone_spi_receive_blocking(uint8_t *pData, uint16_t Size, uint32_t timeout_ms)
{
    return HAL_SPI_Receive(&drone_spi, pData, Size, timeout_ms);
}

HAL_StatusTypeDef drone_spi_transmit_receive_blocking(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t timeout_ms)
{
    return HAL_SPI_TransmitReceive(&drone_spi, pTxData, pRxData, Size, timeout_ms);
}

/* DMA transfer functions */
HAL_StatusTypeDef drone_spi_transmit_dma(uint8_t *pData, uint16_t Size)
{
    return HAL_SPI_Transmit_DMA(&drone_spi, pData, Size);
}

HAL_StatusTypeDef drone_spi_receive_dma(uint8_t *pData, uint16_t Size)
{
    return HAL_SPI_Receive_DMA(&drone_spi, pData, Size);
}

HAL_StatusTypeDef drone_spi_transmit_receive_dma(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    return HAL_SPI_TransmitReceive_DMA(&drone_spi, pTxData, pRxData, Size);
}

/* HAL SPI Callbacks */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == DRONE_SPI_INSTANCE)
    {
        drone_spi_tx_complete_callback();
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == DRONE_SPI_INSTANCE)
    {
        drone_spi_rx_complete_callback();
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == DRONE_SPI_INSTANCE)
    {
        drone_spi_txrx_complete_callback();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == DRONE_SPI_INSTANCE)
    {
        drone_spi_error_callback();
    }
}

HAL_StatusTypeDef drone_spi_wait_txrx_complete(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    while (!spi_txrx_complete)
    {
        if ((HAL_GetTick() - start_tick) > timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }
    spi_txrx_complete = false; // Reset flag for next transfer
    return HAL_OK;
}

HAL_StatusTypeDef drone_spi_wait_tx_complete(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    while (!spi_tx_complete)
    {
        if ((HAL_GetTick() - start_tick) > timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }
    spi_tx_complete = false; // Reset flag for next transfer
    return HAL_OK;
}
/* ========== SPI Manager Task Implementation ========== */

static QueueHandle_t spi_request_queue = NULL;
static TaskHandle_t spi_manager_task_handle = NULL;
static SemaphoreHandle_t spi_dma_semaphore = NULL;
static SpiRequest_t current_request;

/* CS pin control based on device */
static void spi_select_device(DroneSPISensorType_t device)
{
    switch (device)
    {
    case DRONE_SPI_SENSOR_IMU:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // IMU CS low
        break;
    case DRONE_SPI_SENSOR_BAROMETER:
        // Add your barometer CS pin here
        // HAL_GPIO_WritePin(GPIOX, GPIO_PIN_X, GPIO_PIN_RESET);
        break;
    default:
        break;
    }
}

static void spi_deselect_device(DroneSPISensorType_t device)
{
    switch (device)
    {
    case DRONE_SPI_SENSOR_IMU:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // IMU CS high
        break;
    case DRONE_SPI_SENSOR_BAROMETER:
        // Add your barometer CS pin here
        // HAL_GPIO_WritePin(GPIOX, GPIO_PIN_X, GPIO_PIN_SET);
        break;
    default:
        break;
    }
}

/* Override DMA callbacks to use semaphore */
void drone_spi_tx_complete_callback(void)
{
    spi_tx_complete = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(spi_dma_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void drone_spi_rx_complete_callback(void)
{
    spi_txrx_complete = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(spi_dma_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void drone_spi_txrx_complete_callback(void)
{
    spi_txrx_complete = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(spi_dma_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Process a single SPI request */
static void spi_process_request(SpiRequest_t *req)
{
    spi_select_device(req->device);

    switch (req->type)
    {
    case SPI_REQ_READ:
        // Send register address
        spi_tx_buffer[0] = req->reg | 0x80; // Read flag
        spi_tx_complete = false;
        drone_spi_transmit_dma(spi_tx_buffer, 1);
        xSemaphoreTake(spi_dma_semaphore, portMAX_DELAY);

        // Receive data
        spi_txrx_complete = false;
        drone_spi_receive_dma(spi_rx_buffer, req->length);
        xSemaphoreTake(spi_dma_semaphore, portMAX_DELAY);

        // Copy to user buffer
        for (uint16_t i = 0; i < req->length; i++)
        {
            req->rx_data[i] = spi_rx_buffer[i];
        }
        break;

    case SPI_REQ_WRITE:
        // Prepare write command
        spi_tx_buffer[0] = req->reg & 0x7F; // Write flag
        spi_tx_buffer[1] = req->tx_data[0];

        spi_tx_complete = false;
        drone_spi_transmit_dma(spi_tx_buffer, 2);
        xSemaphoreTake(spi_dma_semaphore, portMAX_DELAY);
        break;

    case SPI_REQ_BURST_READ:
        // Send register address
        spi_tx_buffer[0] = req->reg | 0x80; // Read flag
        spi_tx_complete = false;
        drone_spi_transmit_dma(spi_tx_buffer, 1);
        xSemaphoreTake(spi_dma_semaphore, portMAX_DELAY);

        // Receive burst data
        spi_txrx_complete = false;
        drone_spi_receive_dma(spi_rx_buffer, req->length);
        xSemaphoreTake(spi_dma_semaphore, portMAX_DELAY);

        // Copy to user buffer
        for (uint16_t i = 0; i < req->length; i++)
        {
            req->rx_data[i] = spi_rx_buffer[i];
        }
        break;
    }

    spi_deselect_device(req->device);
}

/* SPI Manager Task - processes requests from queue */
static void spi_manager_task(void *pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        // Wait for request from queue
        if (xQueueReceive(spi_request_queue, &current_request, portMAX_DELAY) == pdTRUE)
        {
            // Process the request (blocking with DMA)
            spi_process_request(&current_request);

            // Notify requester that data is ready
            if (current_request.completion != NULL)
            {
                xSemaphoreGive(current_request.completion);
            }
            else if (current_request.requester != NULL)
            {
                xTaskNotifyGive(current_request.requester);
            }
        }
    }
}

/* Initialize SPI Manager Task */
void drone_spi_manager_init(void)
{
    // Create DMA completion semaphore (binary)
    spi_dma_semaphore = xSemaphoreCreateBinary();
    configASSERT(spi_dma_semaphore != NULL);

    // Create request queue (holds 10 requests)
    spi_request_queue = xQueueCreate(10, sizeof(SpiRequest_t));
    configASSERT(spi_request_queue != NULL);

    // Create SPI Manager Task (high priority)
    BaseType_t result = xTaskCreate(
        spi_manager_task,
        "SPI_Mgr",
        256, // Stack size (words)
        NULL,
        configMAX_PRIORITIES - 2, // High priority
        &spi_manager_task_handle);
    configASSERT(result == pdPASS);
}

/* High-level API: Read single register */
HAL_StatusTypeDef spi_manager_read_reg(DroneSPISensorType_t device, uint8_t reg, uint8_t *data, uint32_t timeout_ms)
{
    SemaphoreHandle_t completion_sem = xSemaphoreCreateBinary();
    if (completion_sem == NULL)
    {
        return HAL_ERROR;
    }

    SpiRequest_t request = {
        .device = device,
        .type = SPI_REQ_READ,
        .reg = reg,
        .tx_data = NULL,
        .rx_data = data,
        .length = 1,
        .requester = NULL,
        .completion = completion_sem};

    // Send request to queue
    if (xQueueSend(spi_request_queue, &request, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        vSemaphoreDelete(completion_sem);
        return HAL_TIMEOUT;
    }

    // Wait for completion
    if (xSemaphoreTake(completion_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        vSemaphoreDelete(completion_sem);
        return HAL_TIMEOUT;
    }

    vSemaphoreDelete(completion_sem);
    return HAL_OK;
}

/* High-level API: Write single register */
HAL_StatusTypeDef spi_manager_write_reg(DroneSPISensorType_t device, uint8_t reg, uint8_t data, uint32_t timeout_ms)
{
    SemaphoreHandle_t completion_sem = xSemaphoreCreateBinary();
    if (completion_sem == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t tx_data = data;

    SpiRequest_t request = {
        .device = device,
        .type = SPI_REQ_WRITE,
        .reg = reg,
        .tx_data = &tx_data,
        .rx_data = NULL,
        .length = 1,
        .requester = NULL,
        .completion = completion_sem};

    // Send request to queue
    if (xQueueSend(spi_request_queue, &request, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        vSemaphoreDelete(completion_sem);
        return HAL_TIMEOUT;
    }

    // Wait for completion
    if (xSemaphoreTake(completion_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        vSemaphoreDelete(completion_sem);
        return HAL_TIMEOUT;
    }

    vSemaphoreDelete(completion_sem);
    return HAL_OK;
}

/* High-level API: Burst read multiple registers */
HAL_StatusTypeDef spi_manager_burst_read(DroneSPISensorType_t device, uint8_t reg, uint8_t *buffer, uint16_t length, uint32_t timeout_ms)
{
    SemaphoreHandle_t completion_sem = xSemaphoreCreateBinary();
    if (completion_sem == NULL)
    {
        return HAL_ERROR;
    }

    SpiRequest_t request = {
        .device = device,
        .type = SPI_REQ_BURST_READ,
        .reg = reg,
        .tx_data = NULL,
        .rx_data = buffer,
        .length = length,
        .requester = NULL,
        .completion = completion_sem};

    // Send request to queue
    if (xQueueSend(spi_request_queue, &request, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        vSemaphoreDelete(completion_sem);
        return HAL_TIMEOUT;
    }

    // Wait for completion
    if (xSemaphoreTake(completion_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        vSemaphoreDelete(completion_sem);
        return HAL_TIMEOUT;
    }

    vSemaphoreDelete(completion_sem);
    return HAL_OK;
}

/* Get task handle for monitoring */
TaskHandle_t drone_spi_manager_get_task_handle(void)
{
    return spi_manager_task_handle;
}
