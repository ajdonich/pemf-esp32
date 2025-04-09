#include <Arduino.h>
#include "WssTask.h"

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

WssTask *wssTask = nullptr;
double frequency = 98.0; // Hz
uint64_t toffset = micros();
size_t bytesPerMessage = 128 * (
    sizeof(uint32_t) + 
    sizeof(uint16_t) +
    sizeof(uint16_t)
);

void setFrequency(double freq) {
    frequency = freq;
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

    Serial.printf("Total heap size: %u kb\n", ESP.getHeapSize() / 1024);
    Serial.printf("Free heap: %u kb\n", ESP.getFreeHeap() / 1024);
    Serial.printf("Lowest level of free heap since boot: %u kb\n", ESP.getMinFreeHeap() / 1024);
    Serial.printf("Largest block of heap that can be allocated at once: %u kb\n", ESP.getMaxAllocHeap() / 1024);

    wssTask = new WssTask(setFrequency);
    wssTask->run();

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

    QueueMessage currentMsg(bytesPerMessage);
    while (currentMsg.len < bytesPerMessage) {
        uint32_t usec = (uint32_t)(micros() - toffset);
        uint8_t vindex = getSineIndex(usec, TAU*frequency, 0.6, 1.2);
        vindex += getNoiseIndex(0.0, 0.1);
        safeDacWrite(DAC_PIN, vindex);

        uint16_t sample = analogRead(SIGA_PIN);
        uint16_t sample2 = analogRead(OPTA_PIN);

        currentMsg.append((const void*)&usec, 4);
        currentMsg.append((const void*)&sample, 2);
        currentMsg.append((const void*)&sample2, 2);
        nsamples += 1;
    }

    wssTask->writeMessage(currentMsg);
    // delay(5);
}
