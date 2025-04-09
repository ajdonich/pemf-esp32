#include "Wire.h"
#include "HardwareSerial.h"
#include "ProjectSettings.h"
#include "CmdMsgs.h"
#include "EMDriverTask.h"


TickType_t EMDriverTask::xLockTimout = 2500 / portTICK_PERIOD_MS;

EMDriverTask::EMDriverTask() : EMDriverTask(1022) {}
EMDriverTask::EMDriverTask(uint32_t mHz_a) : EMDriverTask(mHz_a, mHz_a, mHz_a) {}
EMDriverTask::EMDriverTask(uint32_t mHz_a, uint32_t mHz_b, uint32_t mHz_c) 
    : taskHandle(nullptr), freqMutex(nullptr), _mHz{mHz_a, mHz_b, mHz_c}, 
    _omega{(float)(mHz_a * kTAU), (float)(mHz_b * kTAU), (float)(mHz_c * kTAU)} {}

EMDriverTask::~EMDriverTask() {
    if (taskHandle) {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }
}

void EMDriverTask::run() {
    // Set LDAC low will trigger DAC Vout update; can keep
    // low continuously for immediate updates on I2C writes 
    pinMode(LDAC_PIN, OUTPUT);
    digitalWrite(LDAC_PIN, LOW);

    while (true) {
        if (freqMutex = xSemaphoreCreateMutex()) break;
        Serial.println("ERROR: failure in xSemaphoreCreateMutex, unable to create freqMutex");
        delay(1000);
    }

    while (true) {
        if (xTaskCreate(xTaskLoop, "em-driver-task", TASK_STACK_DEPTH, 
            (void*)this, tskHIGH_PRIORITY, &taskHandle) == pdPASS) break;
        Serial.println("ERROR: failure in xTaskCreate, unable to create EMDriverTask");
        delay(1000);
    }
}

void EMDriverTask::setFrequency(const uint32_t mHz_a[3]) {
    if (xSemaphoreTake(freqMutex, xLockTimout ) == pdTRUE ) {
        _mHz[0] = mHz_a[0];
        _mHz[1] = mHz_a[1];
        _mHz[2] = mHz_a[2];
        _omega[0] = _mHz[0] * kTAU;
        _omega[1] = _mHz[1] * kTAU;
        _omega[2] = _mHz[2] * kTAU;
        xSemaphoreGive(freqMutex);
    }
}

void EMDriverTask::setFrequency(const struct EMDriverMsg &msg) {
    if (xSemaphoreTake(freqMutex, xLockTimout ) == pdTRUE ) {
        _mHz[0] = (uint32_t)msg.freq_a;
        _mHz[1] = (uint32_t)msg.freq_b;
        _mHz[2] = (uint32_t)msg.freq_c;
        _omega[0] = _mHz[0] * kTAU;
        _omega[1] = _mHz[1] * kTAU;
        _omega[2] = _mHz[2] * kTAU;
        xSemaphoreGive(freqMutex);
    }
}

void EMDriverTask::getFrequency(uint32_t mHz_a[3]) {
    if (xSemaphoreTake(freqMutex, (TickType_t)0) == pdTRUE ) {
        mHz_a[0] = _mHz[0];
        mHz_a[1] = _mHz[1];
        mHz_a[2] = _mHz[2];
        xSemaphoreGive(freqMutex);
    }
}

void EMDriverTask::getFrequency(struct EMDriverMsg &msg) {
    if (xSemaphoreTake(freqMutex, (TickType_t)0) == pdTRUE ) {
        msg.freq_a = (int32_t)_mHz[0];
        msg.freq_b = (int32_t)_mHz[1];
        msg.freq_c = (int32_t)_mHz[2];
        xSemaphoreGive(freqMutex);
    }
}

void EMDriverTask::getOmega(float omega[3]) {
    if (xSemaphoreTake(freqMutex, (TickType_t)0) == pdTRUE ) {
        omega[0] = _omega[0];
        omega[1] = _omega[1];
        omega[2] = _omega[2];
        xSemaphoreGive(freqMutex);
    }
}

uint16_t EMDriverTask::getSineIndex(uint32_t usec, float omega, float minAmpl, float maxAmpl) {
    float X = std::cos(omega * (float)usec * 1e-6);
    X = X*(maxAmpl - minAmpl) + maxAmpl + minAmpl;
    return (uint16_t)(X * 0.5 * voltage_to_dac);
}

// MCP4728 Write Command: Write Power-Down Selection
uint8_t EMDriverTask::dacPdWrite() {
    static uint8_t cmdBits = 0x5; // C2=1 C1=0 C0=1

    Wire.beginTransmission(DAC_ADDR);
    Wire.write((cmdBits << 5) | (dacPdMode[0] << 2) | dacPdMode[1]);
    Wire.write((dacPdMode[2] << 6) | (dacPdMode[3] << 4));
    return Wire.endTransmission();
}

// MCP4728 Fast Write Command: 
uint8_t EMDriverTask::dacFastWrite(uint32_t oidxmax) {
    // C2=0 C1=0 C0=X (cmdBits unneeded, as they're all 0x0)
    // Note: LDAC must be low to commit this Vout update

    Wire.beginTransmission(DAC_ADDR);
    for (uint32_t i=0; i<oidxmax; ++i) {
        Wire.write((dacPdMode[i] << 4) | (dacCodes[i] >> 8));
        Wire.write(dacCodes[i] & 0xFF);
    }
    return Wire.endTransmission();
}

// MCP4728 Multi-Write Command:
uint8_t EMDriverTask::dacMultiWrite(uint32_t oidxmax) {
    static uint8_t cmdBits = 0x8 << 3;  // C2=0 C1=1 C0=0 W1=0 W0=0
    static uint8_t Vref = 0x0 << 7;     // {0,1} => {Vdd, Vinternal/2.048V}
    static uint8_t Gx = 0x0 << 4;       // {0,1} => {x1, x2} only if Vref == 1

    Wire.beginTransmission(DAC_ADDR);
    for (uint32_t i=0; i<oidxmax; ++i) {
        Wire.write(cmdBits | (i << 1)); // UDAC=0 => Update Vout of this channel
        Wire.write(Vref | (dacPdMode[i] << 5) | Gx | (dacCodes[i] >> 8));
        Wire.write(dacCodes[i] & 0xFF);
    }
    return Wire.endTransmission();
}

void EMDriverTask::logI2CError(uint8_t err) {
    switch (err)
    {
    case 0: Serial.println("I2C write: success"); break;
    case 1: Serial.println("I2C write ERROR: data too long to fit in transmit buffer"); break;
    case 2: Serial.println("I2C write ERROR: received NACK on transmit of address"); break;
    case 3: Serial.println("I2C write ERROR: received NACK on transmit of data"); break;
    case 4: Serial.println("I2C write ERROR: other error"); break;
    case 5: Serial.println("I2C write ERROR: timeout"); break;
    default: Serial.printf("I2C write ERROR: unknown err code %u\n", err); break;
    }
}

// Note: base loop could exceed 105K loops/sec in tests, thus
// the ~2600 samp/sec bottleneck more likely from I2C somehow
void EMDriverTask::xTaskLoop( void *pvParameters ) {
    Serial.println("entered em-driver-task loop");
    EMDriverTask *driverTask = (EMDriverTask*)pvParameters;
    
    Wire.begin(SDA_PIN, SCL_PIN, 100000U);
    Wire.setClock(400000); // I2C fast mode (400k)

    uint8_t err = driverTask->dacPdWrite();
    if (err) driverTask->logI2CError(err);

    uint32_t nsamples = 0;
    uint64_t toffset = micros(); 
    uint64_t tlastreport = micros(); 
    float omega[4] = {1.57, 1.57, 1.57, 0.0};

    uint64_t debugSineTime = 0;
    uint64_t debugWriteTime = 0;

    uint32_t oidxmax = 0;
    while (driverTask->dacPdMode[oidxmax] == 0)
        oidxmax += 1;

    // Task loop
    while (true) {
        uint64_t t1 = micros();
        uint32_t usec = (uint32_t)t1;

        driverTask->getOmega(omega);
        for (uint32_t i=0; i<oidxmax; ++i)
            driverTask->dacCodes[i] = driverTask->getSineIndex(
                usec, omega[i], 0.0, 2.8);

        uint64_t t2 = micros();
        debugSineTime += t2 - t1;

        uint8_t err = driverTask->dacFastWrite(oidxmax);
        if (err) driverTask->logI2CError(err);
        nsamples += 1;

        debugWriteTime += micros() - t2;

        if (micros() - tlastreport > 10e6) {
            double elapsed = (double)(micros() - toffset) * 1e-6;
            double blkelapsed = (micros() - tlastreport) * 1e-6;
            Serial.printf("%.1fs, %.2f samp/sec, sine/write (%.1fs, %.1fs)\n", 
                elapsed, (double)nsamples / blkelapsed,
                (double)(debugSineTime * 1e-6),
                (double)(debugWriteTime * 1e-6));

            tlastreport = micros();
            nsamples = 0;

            debugSineTime = 0;
            debugWriteTime = 0;
        }
    }
}
