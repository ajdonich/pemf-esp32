#include <Arduino.h>
#include "WssTask.h"

// Serial
#define BAUD_RATE 9600

// IO Pins 
#define DAC_PIN 25  // GPIO25 => DAC 1
#define ADC_PIN 32  // GPIO32 => ADC1_4


uint64_t toffset = micros();
size_t bytesPerMessage = 128 * (sizeof(uint32_t) + sizeof(uint16_t));

double frequency = 98.0; // Hz
void setFrequency(double freq) {
    frequency = freq;
}

QueueMessage *currentMsg = new QueueMessage(bytesPerMessage);
WssTask *wssTask = new WssTask(setFrequency);

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) delay(100);
    toffset = micros();
    wssTask->run();
}

void safeDacWrite(uint8_t pin, uint8_t value) {
    if (0 <= value && value <= 255) dacWrite(pin, value);
    else Serial.printf("ERROR: bad dacWrite voltage: %u\n", value);
}

uint8_t getAmplIndex(uint32_t usec, double omega, double minAmpl=0, double maxAmpl=3.3) {
    static float voltage_to_dac = 255.0 / 3.3;
    double X = std::cos(omega * (double)usec * 1e-6);
    X = X*(maxAmpl - minAmpl) + maxAmpl + minAmpl;
    return (uint8_t)round(X * 0.5 * voltage_to_dac);
}

void loop() {
    static double TAU = 2.0*PI;

    while (currentMsg->len < bytesPerMessage) {
        uint32_t usec = (uint32_t)(micros() - toffset);
        uint8_t vindex = getAmplIndex(usec, TAU*frequency, 0.6, 3.2);
        safeDacWrite(DAC_PIN, vindex);

        uint16_t sample = analogRead(ADC_PIN);
        currentMsg->append((const void*)&usec, 4);
        currentMsg->append((const void*)&sample, 2);
    }

    wssTask->writeMessage(*currentMsg);

    delete currentMsg;
    currentMsg = new QueueMessage(bytesPerMessage);
    // delay(5);
}

