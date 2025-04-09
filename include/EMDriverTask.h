
#ifndef EM_DRIVER_TASK_H
#define EM_DRIVER_TASK_H

#include "freertos/task.h"
#include "freertos/semphr.h"


class EMDriverTask {
public:
    EMDriverTask();
    EMDriverTask(uint32_t mHz_a);
    EMDriverTask(uint32_t mHz_a, uint32_t mHz_b, uint32_t mHz_c);
    ~EMDriverTask();

    void run();
    void setFrequency(const uint32_t mHz_a[3]);
    void setFrequency(const struct EMDriverMsg &msg);
    void getFrequency(uint32_t mHz_a[3]);
    void getFrequency(struct EMDriverMsg &msg);

private:
    static void xTaskLoop( void *pvParameters );
    static TickType_t xLockTimout;
    
    void getOmega(float omega[4]);

    uint8_t dacPdWrite();
    uint8_t dacFastWrite(uint32_t oidxmax);
    uint8_t dacMultiWrite(uint32_t oidxmax);
    void logI2CError(uint8_t err);

    uint16_t getSineIndex(uint32_t usec, float omega, 
        float minAmpl=0.0, float maxAmpl=2.8);

    TaskHandle_t taskHandle;
    SemaphoreHandle_t freqMutex;

    // Lock protected frequencies per channel 
    uint32_t _mHz[3];     // mHz (cycle/sec * 1e-3)
    float _omega[3];      // ω (rad/sec)

    // DAC: MCP4728 I2C format spec (see: pg 34, sec 5.5: Writing and Reading Registers and EEPROM):
    // https://datasheet.lcsc.com/lcsc/1811151641_Microchip-Tech-MCP4728-E-UN_C108207.pdf
    const uint8_t DAC_ADDR = 0x60; // 1100 000 (7 bits: 1100 msb always, 000 lsb is default)
    uint8_t dacPdMode[4] = {0, 0, 0, 2}; // Power down: 0, 1, 2, 3 => Normal/On, 1KΩ, 100KΩ, 500KΩ
    uint16_t dacCodes[4] = {0, 0, 0, 0}; // Voltage codes/indexes: [0, 4095]
};

#endif // EM_DRIVER_TASK_H
