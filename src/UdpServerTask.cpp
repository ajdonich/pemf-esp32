#include <Arduino.h>
#include <WiFi.h>
#include "UdpServerTask.h"
#include "ProjectSettings.h"


UdpServerTask::UdpServerTask(freq_callback_t freqCbFcn_a) 
    : freqCbFcn(freqCbFcn_a), udpServerTaskHandle(nullptr), fCount(0) {}

UdpServerTask::~UdpServerTask() {
    if (udpServerTaskHandle) {
        vTaskDelete(udpServerTaskHandle);
        udpServerTaskHandle = nullptr;
    }

    WiFi.disconnect();
    udpServer.stop();
}

void UdpServerTask::run() {
    while (true) {
        if (xTaskCreate(udpServerTaskLoop, "udp-server-task", UDP_STACK_DEPTH, (void*)this, 
            tskIDLE_PRIORITY, &udpServerTaskHandle ) == pdPASS) break;

        Serial.println("ERROR: failure in xTaskCreate, unable to create udpServerTask");
        delay(1000);
    }
}

// Loop will long block on the queue for message arrival
void UdpServerTask::udpServerTaskLoop( void *pvParameters ) {
    Serial.println("entered udp-server-task loop");
    TickType_t xThrottle = 500 / portTICK_PERIOD_MS;
    UdpServerTask *udpServerTask = (UdpServerTask*)pvParameters;
    WiFiUDP* udpServer = &(udpServerTask->udpServer);
    uint16_t MSGSZ = sizeof(UdpFreqMsg);
    char msgBuffer[MSGSZ];

    while (true) {
        udpServerTask->serve();
        int nbytes = udpServer->parsePacket();
        if (nbytes == 0) {
            vTaskDelay(xThrottle);
            continue;
        }

        nbytes = udpServer->read(msgBuffer, MSGSZ);
        if (nbytes != MSGSZ) {
            Serial.printf("WARNING: read invalid nbytes: %d, dropping message\n", nbytes);
            continue;
        }

        UdpFreqMsg fmsg;
        memcpy(&fmsg, msgBuffer, MSGSZ);
        Serial.printf("UdpFreqMsg: (%f, %u, %u) [%d]\n", 
            fmsg.freq, fmsg.emid, fmsg.fcnt, (fmsg.fcnt > udpServerTask->fCount));

        if (fmsg.fcnt > udpServerTask->fCount) {
            udpServerTask->fCount = fmsg.fcnt;
            udpServerTask->freqCbFcn(fmsg.freq, fmsg.emid);
        }
    }
}

void UdpServerTask::serve() {
    if (!WiFi.isConnected()) {
        WiFi.begin(wifissid, wifipword);

        uint32_t elapsed = 0; 
        while (WiFi.status() != WL_CONNECTED) {
            Serial.print("+");
            delay(1000);

            elapsed += 1;
            if (elapsed > 20) {
                Serial.println("terminal WiFi connect timeout, device reset initiated");
                ESP.restart();
            }
        }
        Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString());

        if (udpServer.begin(UDP_PORT) == 0) {
            Serial.println("terminal WiFiUDP begin failure, device reset initiated");
            ESP.restart();
        }
    }
}
