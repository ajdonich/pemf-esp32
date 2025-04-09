#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "SampleTask.h"
#include "WssTask.h"


// Task stack size (in words)
const int STACK_DEPTH = 3072;

SampleTask::SampleTask(int adcpin_a, size_t nsamples_a) : adcpin(adcpin_a), nsamples(nsamples_a) {
    wssTask = new WssTask();
}

void SampleTask::run() {
    wssTask->run();

    while (true) {
        if (xTaskCreate(sampleTaskLoop, "stack-task", STACK_DEPTH, 
            (void*)this, (UBaseType_t)1, &sampleTaskHandle ) == pdPASS) break;

        Serial.println("ERROR: failure in xTaskCreate, unable to create SampleTask");
        delay(1000);
    }
}

// Loop will long block on the queue for message arrival
void SampleTask::sampleTaskLoop( void *pvParameters ) {
    Serial.println("entered sample-task loop");
    SampleTask *sampleTask = (SampleTask*)pvParameters;

    uint16_t samples[sampleTask->nsamples];
    size_t nbytes = sampleTask->nsamples * 2;

    while (true) {
        for (int i = 0; i < sampleTask->nsamples; ++i) {
            samples[i] = analogRead(sampleTask->adcpin);
            delay(50);
        }

        QueueMessage msg((uint8_t *)samples, nbytes);
        sampleTask->wssTask->writeMessage(msg);
    }
}
