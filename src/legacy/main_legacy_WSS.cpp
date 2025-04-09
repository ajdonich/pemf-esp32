#include <Arduino.h>
#include "WssTask.h"

#define BAUD_RATE 9600

WssTask *wssTask;

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) delay(100);
    wssTask = new WssTask();
    wssTask->run();
}

void loop() {
    static int counter = 0;
 
    // This is the text message version
    // QueueMessage msg;
    // msg.len = sprintf(msg.data, "hello world %04d", counter++);
    // wssTask->writeMessage(msg);
    delay(15000);
}
