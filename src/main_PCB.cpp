#include "esp32/clk.h"
#include <Arduino.h>
#include <Wire.h>
#include "TcpServerTask.h"

#include <lwip/sockets.h>
// #include <lwip/netdb.h>


// Serial
#define BAUD_RATE 9600

// GPIO Pins 
#define RDY_PIN 5
#define LDAC_PIN 18
#define SDA_PIN 21
#define SCL_PIN 22
#define SIGA_PIN 34
#define LBATT_PIN 35
#define OPTA_PIN 39

// Consts
#define TAU 6.2831853071795864769252867665590
const double adc_to_voltage = 3.3 / 4095.0;
const double voltage_to_dac = 4095.0 / 3.3;

// See MCP4728 I2C format spec (pg 34, sec 5.5: Writing and Reading Registers and EEPROM):
// https://datasheet.lcsc.com/lcsc/1811151641_Microchip-Tech-MCP4728-E-UN_C108207.pdf

// DAC vars
const int8_t DAC_ADDR = 0x60; // 1100 000 (7 bits: 1100 msb always, 000 lsb is default)
uint8_t dacPdMode[4] = {0, 2, 2, 2}; // Power down: 0, 1, 2, 3 => Normal/On, 1KΩ, 100KΩ, 500KΩ
uint16_t dacCodes[4] = {0, 0, 0, 0}; // Voltage codes/indexes: [0, 4095]

SemaphoreHandle_t freqMutex = nullptr;
TcpServerTask *tcpServerTask = nullptr;
double _frequency = 98.0; // Hz

// Cb for server frequency set 
void setFrequency(double freq, uint32_t emid) {
    static TickType_t xLockTimout = 2500 / portTICK_PERIOD_MS;
    if (xSemaphoreTake(freqMutex, xLockTimout ) == pdTRUE ) {
        _frequency = freq;
        xSemaphoreGive(freqMutex);
    }
}

double getFrequency() {
    static double lastFrequency = 98.0;
    if (xSemaphoreTake(freqMutex, (TickType_t)0) == pdTRUE ) {
        lastFrequency = _frequency;
        xSemaphoreGive(freqMutex);
    }
    return lastFrequency;
}

inline uint32_t telapsed() {
    static uint64_t toffset = micros();
    return (uint32_t)(micros() - toffset);
}

uint16_t getSineIndex(uint32_t usec, double omega, double minAmpl=0.0, double maxAmpl=2.8) {
    double X = std::cos(omega * (double)usec * 1e-6);
    X = X*(maxAmpl - minAmpl) + maxAmpl + minAmpl;
    return (uint16_t)round(X * 0.5 * voltage_to_dac);
}

// MCP4728 Write Command: Write Power-Down Selection
uint8_t dacPdWrite() {
    static uint8_t cmdBits = 0x5 << 5; // C2=1 C1=0 C0=1

    Wire.beginTransmission(DAC_ADDR);
    Wire.write(cmdBits | (dacPdMode[0] << 2) | dacPdMode[1]);
    Wire.write((dacPdMode[2] << 2) | dacPdMode[3]);
    return Wire.endTransmission();
}

// MCP4728 Multi-Write Command: 
uint8_t dacMultiWrite() {
    static uint8_t cmdBits = 0x8 << 3; // C2=0 C1=1 C0=0 W1=0 W0=0
    static uint8_t Vref = 0x0 << 7;    // {0,1} => {Vdd, Vinternal/2.048V}
    static uint8_t Gx = 0x0 << 4;      // {0,1} => {x1, x2} only if Vref == 1

    Wire.beginTransmission(DAC_ADDR);
    for (int i=0; i<4; ++i) {
        if (dacPdMode[i] > 0) continue;
        Wire.write(cmdBits | (i << 1) | 1);
        Wire.write(Vref | (dacPdMode[i] << 6) | Gx | (dacCodes[i] >> 8));
        Wire.write(dacCodes[i] & 0xff);
    }
    return Wire.endTransmission();
}

void logI2CError(uint8_t err) {
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

void logBattLevel() {
    // Battery level on LBATT_PIN is thru 50/50 voltage divider
    double level = adc_to_voltage * 2.0 * analogRead(LBATT_PIN);
    Serial.printf("Battery: %0.2f V\n", level);
}

void logSystemStats() {
    Serial.printf("\nCPU frequency: %u MHz\n", (int)(esp_clk_cpu_freq() / 1e6));
    Serial.printf("FreeRTOS tick rate: %u kHz\n", portTICK_PERIOD_MS);
    Serial.printf("Total heap size: %u kb\n", ESP.getHeapSize() / 1024);
    Serial.printf("Free heap: %u kb\n", ESP.getFreeHeap() / 1024);
    Serial.printf("Lowest level of free heap since boot: %u kb\n", ESP.getMinFreeHeap() / 1024);
    Serial.printf("Largest block of heap that can be allocated at once: %u kb\n", ESP.getMaxAllocHeap() / 1024);
}

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) delay(100);
    Wire.begin(SDA_PIN, SCL_PIN, 100000U);

    struct sockaddr_in svaddr;
    svaddr.sin_addr.s_addr = INADDR_ANY;
    int sfd = socket(AF_INET , SOCK_STREAM, 0);

    uint8_t err = dacPdWrite();
    if (err) logI2CError(err);

    logBattLevel();
    logSystemStats();
    
    while (true) {
        if (freqMutex = xSemaphoreCreateMutex()) break;
        Serial.println("ERROR: failure in xSemaphoreCreateMutex, unable to create freqMutex");
        delay(1000);
    }

    tcpServerTask = new TcpServerTask(setFrequency);
    tcpServerTask->run();

    Serial.printf("Free heap after network init: %u kb\n", ESP.getFreeHeap() / 1024);
}

uint32_t debugSineTime = 0;
uint32_t debugWriteTime = 0;
uint32_t debugLoopTime = 0;

void loop() {
    static uint32_t nsamples = 0;
    static uint32_t tlastreport = telapsed(); 

    uint32_t tloop = telapsed();
    while (telapsed() - tloop < 2e6) {
        uint32_t usec = telapsed();
        uint32_t t1 = usec;

        for (int i=0; i<4; ++i) {
            if (dacPdMode[i] > 0) continue;
            dacCodes[i] = getSineIndex(usec, TAU*getFrequency(), 0.0, 2.8);
        }

        uint32_t t2 = telapsed();
        debugSineTime += t2 - t1;

        uint8_t err = dacMultiWrite();
        if (err) logI2CError(err);
        nsamples += 1;

        debugWriteTime += telapsed() - t2;
    }
    debugLoopTime += telapsed() - tloop;

    if (telapsed() - tlastreport > 10e6) {
        double elapsed = (double)(telapsed() * 1e-6);
        double blkelapsed = (telapsed() - tlastreport) * 1e-6;
        Serial.printf("%.1fs, %.2f samp/sec (%.1fs, %.1fs, %.1fs)\n", 
            elapsed, (double)nsamples / blkelapsed,
            (double)(debugSineTime * 1e-6),
            (double)(debugWriteTime * 1e-6),
            (double)(debugLoopTime * 1e-6));

        tlastreport = telapsed();
        nsamples = 0;

        debugSineTime = 0;
        debugWriteTime = 0;
        debugLoopTime = 0;
    }
    // delay(1);
}
