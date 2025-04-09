#include "esp32/clk.h"
#include <Arduino.h>
#include "UdpServerTask.h"

// Serial
#define BAUD_RATE 9600

// IO Pins 
#define DAC_PIN 25    // GPIO25 => DAC 1
// #define DAC_PIN_2 26  // GPIO26 => DAC 2
// #define ADC_PIN 32    // GPIO32 => ADC1_4

#define SIGA_PIN 34
#define LBATT_PIN 35
#define OPTA_PIN 39

const double voltage_to_dac = 255.0 / 3.3;
const double TAU = 2.0*PI;

SemaphoreHandle_t freqMutex = nullptr;
UdpServerTask *udpServerTask = nullptr;
double _frequency = 98.0; // Hz
uint64_t toffset = micros();

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

void safeDacWrite(uint8_t pin, uint8_t value) {
    if (0 <= value && value <= 255) dacWrite(pin, value);
    else Serial.printf("ERROR: bad dacWrite voltage: %u\n", value);
}

uint8_t getSineIndex(uint32_t usec, double omega, double minAmpl=0, double maxAmpl=3.3) {
    double X = std::cos(omega * (double)usec * 1e-6);
    X = X*(maxAmpl - minAmpl) + maxAmpl + minAmpl;
    return (uint8_t)round(X * 0.5 * voltage_to_dac);
}

uint8_t getNoiseIndex(double minAmpl=0, double maxAmpl=3.3) {
    double X = (double)esp_random() / UINT32_MAX * (maxAmpl-minAmpl);
    return (uint8_t)((X + minAmpl) * voltage_to_dac);
}

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) delay(100);
    toffset = micros();

    Serial.printf("\nCPU frequency: %u MHz\n", (int)(esp_clk_cpu_freq() / 1e6));
    Serial.printf("FreeRTOS tick rate: %u kHz\n", portTICK_PERIOD_MS);
    Serial.printf("Total heap size: %u kb\n", ESP.getHeapSize() / 1024);
    Serial.printf("Free heap: %u kb\n", ESP.getFreeHeap() / 1024);
    Serial.printf("Lowest level of free heap since boot: %u kb\n", ESP.getMinFreeHeap() / 1024);
    Serial.printf("Largest block of heap that can be allocated at once: %u kb\n", ESP.getMaxAllocHeap() / 1024);

    while (true) {
        if (freqMutex = xSemaphoreCreateMutex()) break;
        Serial.println("ERROR: failure in xSemaphoreCreateMutex, unable to create freqMutex");
        delay(1000);
    }

    udpServerTask = new UdpServerTask(setFrequency);
    udpServerTask->run();

    Serial.printf("Free heap after network init: %u kb\n", ESP.getFreeHeap() / 1024);
}

void loop() {
    static uint32_t nsamples = 0;
    static uint32_t lastreport = (micros() - toffset);

    if (micros() - lastreport > 10e6) {
        uint32_t elapsed = (uint32_t)(micros() - toffset) * 1e-6;
        uint32_t blkelapsed = (uint32_t)(micros() - lastreport) * 1e-6;
        Serial.printf("%us, %.2f samp/sec\n", elapsed, (double)nsamples / blkelapsed);        
        lastreport = micros();
        nsamples = 0;
    }

    while (micros() - lastreport < 2e6) {
        uint32_t usec = (uint32_t)(micros() - toffset);
        uint8_t vindex = getSineIndex(usec, TAU*getFrequency(), 0.6, 1.2);
        vindex += getNoiseIndex(0.0, 0.1);
        safeDacWrite(DAC_PIN, vindex);
        nsamples += 1;
    }

    // delay(5);
}
