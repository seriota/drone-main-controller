# Initialization Sequence - Synchronous Init, Async Runtime

## 🎯 Architecture

This approach uses **blocking SPI** during system initialization (before RTOS starts), then switches to **async DMA-based SPI** during runtime.

### Benefits
✅ Simple initialization - no need for semaphores/queues during boot  
✅ Fast init - CPU can busy-wait during short init phase  
✅ Efficient runtime - DMA + RTOS for maximum performance  
✅ Clean separation - init code vs runtime code  

---

## 📝 Complete Example (main.c)

```c
#include "main.h"
#include "drone_spi_communication.h"
#include "drone_imu.h"
#include "FreeRTOS.h"
#include "task.h"

/* Task prototypes */
void imu_task(void *pvParameters);
void baro_task(void *pvParameters);
void control_task(void *pvParameters);

int main(void)
{
    /* ========== PHASE 1: Hardware Init (Blocking SPI) ========== */
    
    HAL_Init();
    SystemClock_Config();
    
    // Initialize SPI peripheral (DMA interrupts DISABLED)
    drone_spi_init();
    
    /* ========== Initialize IMU (Blocking/Synchronous) ========== */
    
    // CS pin setup
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_4;  // IMU CS
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // CS high
    
    // Verify IMU WHO_AM_I (blocking)
    uint8_t tx_buf[2], rx_buf[2];
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // CS low
    tx_buf[0] = 0x75 | 0x80;  // WHO_AM_I register with read flag
    drone_spi_transmit_blocking(tx_buf, 1, 100);
    drone_spi_receive_blocking(rx_buf, 1, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // CS high
    
    if (rx_buf[0] != 0x71) {
        Error_Handler();  // IMU not detected
    }
    
    // Configure IMU registers (blocking)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    tx_buf[0] = 0x6B;  // PWR_MGMT_1
    tx_buf[1] = 0x01;  // Wake up, use PLL
    drone_spi_transmit_blocking(tx_buf, 2, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    
    HAL_Delay(10);  // Wait for IMU to wake up
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    tx_buf[0] = 0x1B;  // GYRO_CONFIG
    tx_buf[1] = 0x08;  // ±500 dps
    drone_spi_transmit_blocking(tx_buf, 2, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    tx_buf[0] = 0x1C;  // ACCEL_CONFIG
    tx_buf[1] = 0x08;  // ±4g
    drone_spi_transmit_blocking(tx_buf, 2, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    
    /* ========== Initialize Barometer (Blocking/Synchronous) ========== */
    
    // Setup barometer CS pin
    GPIO_InitStruct.Pin = GPIO_PIN_0;  // Baro CS (example)
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    
    // Configure barometer...
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    tx_buf[0] = 0x26;  // Example: CTRL_REG1
    tx_buf[1] = 0x39;  // Example: Power up, OSR = 128
    drone_spi_transmit_blocking(tx_buf, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    
    HAL_Delay(100);  // Wait for barometer ready
    
    /* ========== PHASE 2: Switch to Async Mode ========== */
    
    // Enable DMA interrupts for async operation
    drone_spi_enable_dma_interrupts();
    
    // Create SPI Manager Task
    drone_spi_manager_init();
    
    /* ========== PHASE 3: Create Application Tasks ========== */
    
    xTaskCreate(imu_task, "IMU", 512, NULL, 4, NULL);
    xTaskCreate(baro_task, "Baro", 256, NULL, 2, NULL);
    xTaskCreate(control_task, "Control", 512, NULL, 3, NULL);
    
    /* ========== PHASE 4: Start RTOS ========== */
    
    vTaskStartScheduler();
    
    /* Should never reach here */
    while (1) {
        Error_Handler();
    }
}

/* ========== Runtime Tasks (Async SPI via Manager) ========== */

void imu_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t gyro_data[6];
    
    while (1)
    {
        // Use async SPI Manager (task sleeps during transfer)
        HAL_StatusTypeDef status = spi_manager_burst_read(
            DRONE_SPI_SENSOR_IMU,
            0x43,  // GYRO_XOUT_H
            gyro_data,
            6,
            100
        );
        
        if (status == HAL_OK)
        {
            int16_t gyro_x = (gyro_data[0] << 8) | gyro_data[1];
            int16_t gyro_y = (gyro_data[2] << 8) | gyro_data[3];
            int16_t gyro_z = (gyro_data[4] << 8) | gyro_data[5];
            
            // Process gyro data...
        }
        
        // 1000 Hz (1ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

void baro_task(void *pvParameters)
{
    uint8_t pressure_data[3];
    
    while (1)
    {
        // Read pressure (async)
        HAL_StatusTypeDef status = spi_manager_burst_read(
            DRONE_SPI_SENSOR_BAROMETER,
            0x01,  // Example: Pressure register
            pressure_data,
            3,
            100
        );
        
        if (status == HAL_OK)
        {
            uint32_t pressure = (pressure_data[0] << 16) | 
                               (pressure_data[1] << 8) | 
                                pressure_data[2];
            // Process pressure...
        }
        
        // 50 Hz (20ms)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void control_task(void *pvParameters)
{
    while (1)
    {
        // PID control, motor output, etc.
        
        vTaskDelay(pdMS_TO_TICKS(2));  // 500 Hz
    }
}
```

---

## 🔄 Detailed Flow

### Phase 1: Initialization (Synchronous)

```
main() starts
    ↓
drone_spi_init()
    ├─ Configure SPI peripheral
    ├─ Configure DMA
    └─ DMA interrupts DISABLED ← Still blocking mode
    ↓
Configure IMU (blocking SPI)
    ├─ WHO_AM_I verification
    ├─ Power management
    ├─ Gyro configuration
    └─ Accel configuration
    ↓
Configure Barometer (blocking SPI)
    ├─ Power up
    ├─ Set OSR
    └─ Enable measurements
```

**Characteristics:**
- CPU busy-waits during transfers (OK, init is fast)
- No interrupts, no callbacks
- Simple sequential code
- No RTOS overhead

### Phase 2: Enable Async Mode

```
drone_spi_enable_dma_interrupts()
    ├─ Enable TX interrupt
    └─ Enable RX interrupt
    ↓
drone_spi_manager_init()
    ├─ Create DMA semaphore
    ├─ Create request queue
    └─ Create SPI Manager Task
```

### Phase 3: Runtime (Async)

```
Tasks call spi_manager_burst_read()
    ↓
Request queued
Task SLEEPS (not busy-waiting!)
    ↓
SPI Manager processes request
    ├─ DMA transfers data
    ├─ Task sleeps on DMA semaphore
    └─ DMA interrupt wakes manager
    ↓
Requesting task wakes with data
```

**Characteristics:**
- Tasks sleep during transfers
- CPU free for other tasks
- DMA + interrupts handle transfers
- Maximum efficiency

---

## 📊 Comparison

| Phase | Mode | CPU Usage | When |
|-------|------|-----------|------|
| **Init** | Blocking | High (busy-wait) | Boot only (~100ms) |
| **Runtime** | Async DMA | Low (sleeping) | Continuous operation |

---

## 🎯 Alternative: Pure Blocking Init

If you prefer even simpler init code, you can use the existing IMU functions:

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    drone_spi_init();  // DMA interrupts disabled
    
    // Use existing drone_imu_init() - it will use blocking internally
    // Just need to temporarily modify it to use blocking functions
    drone_imu_init();
    
    // Initialize barometer...
    
    // Switch to async
    drone_spi_enable_dma_interrupts();
    drone_spi_manager_init();
    
    // Create tasks...
    xTaskCreate(...);
    
    vTaskStartScheduler();
}
```

---

## ⚠️ Important Notes

1. **Call `drone_spi_enable_dma_interrupts()` BEFORE `drone_spi_manager_init()`**  
   The manager task needs interrupts enabled to work.

2. **Don't mix blocking and async SPI**  
   Once RTOS starts, use only the SPI Manager functions.

3. **Initialization is fast**  
   Busy-waiting during init is acceptable since it only happens once at boot.

4. **Error handling**  
   Check return values during init - if a sensor fails, you know immediately.

---

## 🚀 Summary

Your plan is **perfect**! This approach gives you:

✅ **Simple init code** - No RTOS complexity during boot  
✅ **Reliable init** - Blocking ensures sensors are ready  
✅ **Efficient runtime** - Full DMA benefits during flight  
✅ **Clean architecture** - Clear separation of concerns  

Just remember the sequence:
1. `drone_spi_init()` (interrupts disabled)
2. Configure sensors with blocking SPI
3. `drone_spi_enable_dma_interrupts()` ← KEY STEP!
4. `drone_spi_manager_init()`
5. Create tasks
6. `vTaskStartScheduler()`
