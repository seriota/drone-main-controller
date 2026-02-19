#ifndef DRONE_SPI_COMMUNICATION_H
#define DRONE_SPI_COMMUNICATION_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "helper.h"

#define DRONE_SPI_INSTANCE SPI1
#define DRONE_SCK_PIN GPIO_PIN_5
#define DRONE_MISO_PIN GPIO_PIN_6
#define DRONE_MOSI_PIN GPIO_PIN_7
#define DRONE_SPI_GPIO GPIOA
#define DRONE_ENABLE_SPI_GPIO_CLK() __HAL_RCC_GPIOA_CLK_ENABLE()
#define DRONE_ENABLE_SPI_CLK() __HAL_RCC_SPI1_CLK_ENABLE()
#define DRONE_ENABLE_DMA_CLK() __HAL_RCC_DMA2_CLK_ENABLE()

/* DMA Stream/Channel definitions for SPI1 */
#define DRONE_SPI_TX_DMA_STREAM DMA2_Stream3
#define DRONE_SPI_RX_DMA_STREAM DMA2_Stream0 /* Changed from Stream2 to avoid TIM1 conflict */
#define DRONE_SPI_DMA_CHANNEL DMA_CHANNEL_3

/* DMA IRQ numbers */
#define DRONE_SPI_TX_DMA_IRQn DMA2_Stream3_IRQn
#define DRONE_SPI_RX_DMA_IRQn DMA2_Stream0_IRQn /* Changed from Stream2_IRQn */

typedef enum
{
    DRONE_SPI_NO_SENSOR = 0x00,
    DRONE_SPI_SENSOR_IMU,
    DRONE_SPI_SENSOR_BAROMETER,
} DroneSPISensorType_t;

/* SPI Request types */
typedef enum
{
    SPI_REQ_READ,       // Read register(s)
    SPI_REQ_WRITE,      // Write register
    SPI_REQ_BURST_READ, // Burst read multiple bytes
} SpiRequestType_t;

/* SPI Request structure */
typedef struct
{
    DroneSPISensorType_t device;  // Which device (IMU, Baro)
    SpiRequestType_t type;        // Read/Write/Burst
    uint8_t reg;                  // Register address
    uint8_t *tx_data;             // TX data buffer (for write)
    uint8_t *rx_data;             // RX data buffer (where to store result)
    uint16_t length;              // Number of bytes
    TaskHandle_t requester;       // Task to notify when done
    SemaphoreHandle_t completion; // Semaphore to signal completion
} SpiRequest_t;

extern SPI_HandleTypeDef drone_spi;
extern DMA_HandleTypeDef drone_spi_tx_dma;
extern DMA_HandleTypeDef drone_spi_rx_dma;

/* DMA buffers and flags */
extern uint8_t spi_tx_buffer[32];
extern uint8_t spi_rx_buffer[32];
extern volatile bool spi_txrx_complete;
extern volatile bool spi_tx_complete;
extern volatile DroneSPISensorType_t current_spi_sensor; // Track which sensor is currently being accessed

void drone_spi_init(void);
void drone_spi_dma_init(void);
bool is_spi_initialized(void);

/* DMA interrupt control */
void drone_spi_enable_dma_interrupts(void);
void drone_spi_disable_dma_interrupts(void);

/* DMA transfer functions */
HAL_StatusTypeDef drone_spi_transmit_dma(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef drone_spi_receive_dma(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef drone_spi_transmit_receive_dma(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

/* Synchronous (blocking) SPI functions - for initialization */
HAL_StatusTypeDef drone_spi_transmit_blocking(uint8_t *pData, uint16_t Size, uint32_t timeout_ms);
HAL_StatusTypeDef drone_spi_receive_blocking(uint8_t *pData, uint16_t Size, uint32_t timeout_ms);
HAL_StatusTypeDef drone_spi_transmit_receive_blocking(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t timeout_ms);

/* Callback functions (weak - can be overridden by user) */
void drone_spi_tx_complete_callback(void);
void drone_spi_rx_complete_callback(void);
void drone_spi_txrx_complete_callback(void);
void drone_spi_error_callback(void);

/* Helper functions to wait for DMA completion */
HAL_StatusTypeDef drone_spi_wait_txrx_complete(uint32_t timeout_ms);
HAL_StatusTypeDef drone_spi_wait_tx_complete(uint32_t timeout_ms);

/* ========== SPI Manager Task API ========== */

/* Initialize SPI Manager Task */
void drone_spi_manager_init(void);

/* High-level request functions - blocking until complete */
HAL_StatusTypeDef spi_manager_read_reg(DroneSPISensorType_t device, uint8_t reg, uint8_t *data, uint32_t timeout_ms);
HAL_StatusTypeDef spi_manager_write_reg(DroneSPISensorType_t device, uint8_t reg, uint8_t data, uint32_t timeout_ms);
HAL_StatusTypeDef spi_manager_burst_read(DroneSPISensorType_t device, uint8_t reg, uint8_t *buffer, uint16_t length, uint32_t timeout_ms);

/* Get the SPI Manager task handle (for debugging/monitoring) */
TaskHandle_t drone_spi_manager_get_task_handle(void);

#endif