# SPI Manager Task - Usage Guide

## 🎯 Overview

The SPI Manager Task provides **thread-safe, DMA-based SPI communication** for multiple sensors on a shared SPI bus using FreeRTOS.

### Key Features
✅ **Thread-safe** - Multiple tasks can safely request SPI operations  
✅ **DMA-based** - Tasks sleep during transfers (not busy-waiting)  
✅ **Automatic serialization** - Requests processed one at a time  
✅ **Simple API** - Easy blocking calls that return data

---

## 🚀 Quick Start

### 1. Initialize (in main.c, before starting scheduler)

```c
#include "drone_spi_communication.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    // Initialize SPI hardware
    drone_spi_init();
    
    // Create SPI Manager Task
    drone_spi_manager_init();  // ← Creates task & queue
    
    // Create your application tasks
    xTaskCreate(imu_task, "IMU", 256, NULL, 3, NULL);
    xTaskCreate(baro_task, "Baro", 256, NULL, 2, NULL);
    
    // Start scheduler
    vTaskStartScheduler();
    
    while (1) {}
}
```

---

## 📖 API Functions

### **Read Single Register**
```c
HAL_StatusTypeDef spi_manager_read_reg(
    DroneSPISensorType_t device,  // DRONE_SPI_SENSOR_IMU or DRONE_SPI_SENSOR_BAROMETER
    uint8_t reg,                  // Register address
    uint8_t *data,                // Where to store result
    uint32_t timeout_ms           // Timeout in milliseconds
);
```

### **Write Single Register**
```c
HAL_StatusTypeDef spi_manager_write_reg(
    DroneSPISensorType_t device,
    uint8_t reg,
    uint8_t data,                 // Data to write
    uint32_t timeout_ms
);
```

### **Burst Read Multiple Registers**
```c
HAL_StatusTypeDef spi_manager_burst_read(
    DroneSPISensorType_t device,
    uint8_t reg,                  // Starting register
    uint8_t *buffer,              // Buffer to store data
    uint16_t length,              // Number of bytes to read
    uint32_t timeout_ms
);
```

---

## 💡 Usage Examples

### Example 1: IMU Task (High-frequency sensor reading)

```c
#include "drone_spi_communication.h"
#include "drone_imu.h"

void imu_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t gyro_data[6];
    DroneImuRaw_t gyro_raw;
    
    while (1)
    {
        // Read 6 bytes starting at GYRO_XOUT_H register
        HAL_StatusTypeDef status = spi_manager_burst_read(
            DRONE_SPI_SENSOR_IMU,           // Device
            MPU9250_REG_GYRO_XOUT_H,        // Register
            gyro_data,                       // Buffer
            6,                               // Length
            100                              // Timeout 100ms
        );
        
        if (status == HAL_OK)
        {
            // Parse data
            gyro_raw.x = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
            gyro_raw.y = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
            gyro_raw.z = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);
            
            // Use gyro data...
            // Send to control task, etc.
        }
        
        // Run at 1000 Hz (1ms period)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
```

### Example 2: Barometer Task (Low-frequency sensor)

```c
void baro_task(void *pvParameters)
{
    uint8_t pressure_data[3];
    
    // Initialize barometer
    spi_manager_write_reg(DRONE_SPI_SENSOR_BAROMETER, BARO_CTRL_REG, 0x01, 100);
    
    while (1)
    {
        // Start conversion
        spi_manager_write_reg(DRONE_SPI_SENSOR_BAROMETER, BARO_CMD_REG, 0x40, 100);
        
        // Wait for conversion
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // Read pressure data
        HAL_StatusTypeDef status = spi_manager_burst_read(
            DRONE_SPI_SENSOR_BAROMETER,
            BARO_PRESS_REG,
            pressure_data,
            3,
            100
        );
        
        if (status == HAL_OK)
        {
            uint32_t pressure = (pressure_data[0] << 16) | 
                               (pressure_data[1] << 8) | 
                                pressure_data[2];
            // Use pressure data...
        }
        
        // Run at 50 Hz (20ms period)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```

### Example 3: Configuration/Initialization

```c
void sensor_init_task(void *pvParameters)
{
    // Configure IMU
    spi_manager_write_reg(DRONE_SPI_SENSOR_IMU, MPU9250_REG_PWR_MGMT_1, 0x01, 100);
    spi_manager_write_reg(DRONE_SPI_SENSOR_IMU, MPU9250_REG_GYRO_CONFIG, 0x08, 100);
    spi_manager_write_reg(DRONE_SPI_SENSOR_IMU, MPU9250_REG_ACCEL_CONFIG, 0x08, 100);
    
    // Verify IMU WHO_AM_I
    uint8_t who_am_i;
    spi_manager_read_reg(DRONE_SPI_SENSOR_IMU, MPU9250_REG_WHO_AM_I, &who_am_i, 100);
    
    if (who_am_i == 0x71)
    {
        // IMU detected successfully
    }
    
    // Configure barometer...
    
    // Task can delete itself after initialization
    vTaskDelete(NULL);
}
```

---

## 🔧 How It Works Internally

```
┌─────────────┐                    ┌─────────────────────┐
│  IMU Task   │                    │  SPI Manager Task   │
│             │                    │  (Priority: High)   │
└──────┬──────┘                    └──────────┬──────────┘
       │                                      │
       │ 1. Call spi_manager_burst_read()    │
       │    Creates request                  │
       │                                      │
       │ 2. Send request to queue ─────────▶ │
       │                                      │
       │ 3. Task SLEEPS on semaphore         │ 4. Process request:
       │    (not busy-waiting!)              │    - Select CS
       │                                      │    - Start DMA TX
       │                                      │    - Wait on DMA sem
       │                          ┌───────────┤    - Start DMA RX
       │                          │ DMA       │    - Wait on DMA sem
       │                          │ Transfer  │    - Deselect CS
       │                          │ Complete  │    - Copy data
       │                          └───────────┤
       │ 5. Wake up! Data ready  ◀───────────┤ 6. Signal completion
       │                                      │
       │ 6. Return data to caller            │ 7. Wait for next request
       │                                      │
       └─────────────────────────────────────┴───────────────────┘
```

### Key Points:
- **Requesting task sleeps** on a semaphore (not busy-waiting)
- **SPI Manager sleeps** on DMA semaphore during transfers (not busy-waiting)
- **CPU is free** to run other tasks during SPI transfers
- **Automatic serialization** prevents conflicts

---

## ⚙️ Configuration

### Add Barometer CS Pin

Edit the CS pin functions in `drone_spi_communication.c`:

```c
static void spi_select_device(DroneSPISensorType_t device)
{
    switch (device) {
        case DRONE_SPI_SENSOR_IMU:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            break;
        case DRONE_SPI_SENSOR_BAROMETER:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // ← Add your pin
            break;
    }
}

static void spi_deselect_device(DroneSPISensorType_t device)
{
    switch (device) {
        case DRONE_SPI_SENSOR_IMU:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            break;
        case DRONE_SPI_SENSOR_BAROMETER:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // ← Add your pin
            break;
    }
}
```

### Adjust Queue Size

In `drone_spi_manager_init()`:
```c
// Create request queue (10 requests max)
spi_request_queue = xQueueCreate(10, sizeof(SpiRequest_t));  // ← Change size
```

### Adjust Task Priority/Stack

In `drone_spi_manager_init()`:
```c
xTaskCreate(
    spi_manager_task,
    "SPI_Mgr",
    256,                           // ← Stack size (words)
    NULL,
    configMAX_PRIORITIES - 2,      // ← Priority
    &spi_manager_task_handle
);
```

---

## 📊 Performance Benefits

| Metric | Before (Blocking) | After (SPI Manager) |
|--------|-------------------|---------------------|
| CPU during SPI transfer | 100% busy-waiting | ~0% (sleeping) |
| Multi-sensor support | Unsafe | Thread-safe |
| Code complexity | Low | Medium |
| Efficiency | Poor | Excellent |

---

## ⚠️ Important Notes

1. **Initialize before scheduler**: Call `drone_spi_manager_init()` before `vTaskStartScheduler()`

2. **Timeout values**: Set realistic timeouts (100ms is usually safe)

3. **Queue full**: If queue fills up (10 requests), new requests will timeout

4. **Priority**: SPI Manager Task should be high priority to minimize latency

5. **Stack size**: 256 words should be sufficient, increase if using complex sensors

6. **CS pins**: Don't forget to initialize CS GPIO pins as outputs before using!

---

## 🐛 Troubleshooting

### Request times out
- Check SPI Manager Task is running
- Check CS pins are configured correctly
- Increase timeout value
- Check device is responding

### Data corruption
- Verify SPI clock settings match sensor requirements
- Check wire connections
- Ensure CS timing is correct

### Queue full errors
- Increase queue size in init
- Reduce request frequency
- Check SPI Manager Task priority

---

## 🎯 Summary

**Simple 3-step usage:**
1. Call `drone_spi_manager_init()` once at startup
2. Use `spi_manager_read_reg()` / `spi_manager_burst_read()` from any task
3. Data is automatically returned - no callbacks needed!

**Behind the scenes:**
- Your task sleeps (not busy-waiting)
- SPI Manager handles DMA
- DMA controller transfers data
- You wake up with data ready

**Result:** Maximum CPU efficiency with simple blocking API! 🚀
