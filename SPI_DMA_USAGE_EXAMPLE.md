# SPI1 DMA Implementation Guide

## Overview
DMA (Direct Memory Access) has been implemented for SPI1 to enable non-blocking data transfers. This implementation uses:
- **DMA2 Stream 3, Channel 3** → SPI1_TX (Transmit)
- **DMA2 Stream 2, Channel 3** → SPI1_RX (Receive)

## Key Features
✓ Non-blocking SPI transfers
✓ Interrupt-driven completion callbacks
✓ Automatic DMA initialization
✓ Error handling callbacks
✓ Configurable priority levels (currently set to 5)

## Usage Examples

### 1. Basic Transmit (TX only)
```c
uint8_t txBuffer[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};

// Start DMA transfer
if (drone_spi_transmit_dma(txBuffer, sizeof(txBuffer)) == HAL_OK)
{
    // Transfer started successfully
    // Your code continues here (non-blocking)
}

// Implement callback to know when transfer is complete
void drone_spi_tx_complete_callback(void)
{
    // TX transfer completed
    // Add your post-transfer code here
}
```

### 2. Basic Receive (RX only)
```c
uint8_t rxBuffer[10];

// Start DMA receive
if (drone_spi_receive_dma(rxBuffer, sizeof(rxBuffer)) == HAL_OK)
{
    // Receive started successfully
}

// Implement callback to handle received data
void drone_spi_rx_complete_callback(void)
{
    // RX transfer completed
    // Process received data in rxBuffer
}
```

### 3. Full Duplex (TX + RX simultaneously)
```c
uint8_t txBuffer[10] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22, 0x33, 0x44};
uint8_t rxBuffer[10];

// Start simultaneous TX/RX
if (drone_spi_transmit_receive_dma(txBuffer, rxBuffer, sizeof(txBuffer)) == HAL_OK)
{
    // Transfer started successfully
}

// Implement callback for completion
void drone_spi_txrx_complete_callback(void)
{
    // Both TX and RX completed
    // Process rxBuffer here
}
```

### 4. Error Handling
```c
void drone_spi_error_callback(void)
{
    // DMA transfer error occurred
    // Check error flags:
    uint32_t error = HAL_SPI_GetError(&drone_spi);
    
    if (error & HAL_SPI_ERROR_DMA)
    {
        // DMA transfer error
    }
    if (error & HAL_SPI_ERROR_OVR)
    {
        // Overrun error
    }
    // Handle or log error
}
```

## Integration Steps

### Step 1: Initialize SPI with DMA
```c
void setup(void)
{
    SystemClock_Config();
    
    // This automatically initializes both SPI and DMA
    drone_spi_init();
    
    // SPI is now ready for DMA transfers
}
```

### Step 2: Implement Callbacks (Optional but Recommended)
Create these functions in your application code to handle transfer completion:

```c
// In your main.c or application file
void drone_spi_tx_complete_callback(void)
{
    // Called when TX DMA transfer completes
}

void drone_spi_rx_complete_callback(void)
{
    // Called when RX DMA transfer completes
}

void drone_spi_txrx_complete_callback(void)
{
    // Called when full-duplex transfer completes
}

void drone_spi_error_callback(void)
{
    // Called on DMA/SPI error
}
```

## Important Notes

### Memory Considerations
- **DMA buffers MUST be in RAM** (not flash/const)
- Buffers should be properly aligned
- Do NOT modify buffers during transfer
- For safety with FreeRTOS, consider using DMA-safe memory regions

### Chip Select (CS) Management
Since NSS is configured as `SPI_NSS_SOFT`, you need to manage CS manually:

```c
// Example with manual CS control
#define CS_PIN GPIO_PIN_4
#define CS_PORT GPIOA

// Pull CS low before transfer
HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

// Start DMA transfer
drone_spi_transmit_receive_dma(txData, rxData, dataSize);

// Wait for completion in callback, then raise CS
void drone_spi_txrx_complete_callback(void)
{
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}
```

### DMA Priority Configuration
Current DMA interrupt priority is set to **5** (preemption priority).
Adjust in `drone_spi_dma_init()` if needed:
```c
HAL_NVIC_SetPriority(DRONE_SPI_TX_DMA_IRQn, 5, 0); // Change 5 to your priority
HAL_NVIC_SetPriority(DRONE_SPI_RX_DMA_IRQn, 5, 0); // Change 5 to your priority
```

### FreeRTOS Integration
When using with FreeRTOS, ensure interrupt priorities are compatible:
- FreeRTOS `configMAX_SYSCALL_INTERRUPT_PRIORITY` is typically 5
- DMA interrupts at priority 5 can call FreeRTOS API functions
- Lower numeric values = higher priority

## Performance Benefits
- **Non-blocking**: CPU is free during transfer
- **Efficient**: No polling loops consuming CPU cycles
- **Fast**: Hardware handles data transfer
- **Scalable**: Easy to transfer large buffers

## Troubleshooting

### Transfer doesn't complete
- Check DMA clock is enabled
- Verify interrupt handlers are properly linked
- Ensure buffers are in RAM, not flash
- Check buffer size isn't zero

### Data corruption
- Verify SPI clock settings (Mode 3: CPOL=1, CPHA=1)
- Check CS timing
- Ensure buffer isn't modified during transfer
- Verify cable/connection quality

### Callbacks not firing
- Ensure DMA interrupts are enabled in NVIC
- Check interrupt priority doesn't conflict with FreeRTOS
- Verify `HAL_DMA_IRQHandler()` is being called

## Reference: STM32F4 DMA Mapping for SPI1
| Peripheral | Direction | DMA Controller | Stream | Channel |
|------------|-----------|----------------|---------|---------|
| SPI1_TX    | Memory→SPI | DMA2 | Stream 3 | Channel 3 |
| SPI1_RX    | SPI→Memory | DMA2 | Stream 2 | Channel 3 |
| (Alt) SPI1_RX | SPI→Memory | DMA2 | Stream 0 | Channel 3 |

*Current implementation uses Stream 2 for RX*
