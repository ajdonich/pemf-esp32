#include <SPI.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include "TcpServerTask.h"
#include "ProjectSettings.h"


TickType_t TcpServerTask::xThrottle = 500 / portTICK_PERIOD_MS;

TcpServerTask::TcpServerTask(freq_update_cb freqCbFcn_a) 
    : freqCbFcn(freqCbFcn_a), tcpServerTaskHandle(nullptr), fCount(0) {}

TcpServerTask::~TcpServerTask() {
    if (tcpServerTaskHandle) {
        vTaskDelete(tcpServerTaskHandle);
        tcpServerTaskHandle = nullptr;
    }

    WiFi.disconnect();
}

void TcpServerTask::run() {
    while (true) {
        if (xTaskCreate(tcpServerTaskLoop, "tcp-server-task", TCP_STACK_DEPTH, (void*)this, 
            tskIDLE_PRIORITY, &tcpServerTaskHandle ) == pdPASS) break;

        Serial.println("ERROR: failure in xTaskCreate, unable to create tcpServerTask");
        delay(1000);
    }
}

// Note: for some reason have to fully refresh the WiFiServer listen
// socket to accept a subsequent client connection (after a disconnect)
WiFiClient TcpServerTask::awaitConnection() {
    WiFiServer listenServer;
    listenServer.begin(TCP_PORT);
    WiFiClient client = listenServer.accept();
    
    while (!client) {   
        client = listenServer.accept();
        vTaskDelay(xThrottle);
    }

    Serial.println("Client connection accepted");
    listenServer.end();
    return client;
}

// Loop will long block on the queue for message arrival
void TcpServerTask::tcpServerTaskLoop( void *pvParameters ) {
    Serial.println("entered tcp-server-task loop");
    TcpServerTask *tcpServerTask = (TcpServerTask*)pvParameters;
    uint16_t MSGSZ = sizeof(FreqUpdateMsg);
    char msgBuffer[MSGSZ];

    while (true) {
        tcpServerTask->validateWiFi();
        WiFiClient client = awaitConnection();

        while (client) {
            if (client.available() > 0) {
                size_t nbytes = client.readBytes(msgBuffer, MSGSZ);
                if (nbytes != MSGSZ) {
                    Serial.printf("WARNING: invalid nbytes: %d != %d, dropping msg\n", nbytes, MSGSZ);
                    client.flush();
                    continue;
                }

                FreqUpdateMsg fmsg;
                memcpy(&fmsg, msgBuffer, MSGSZ);
                Serial.printf("FreqUpdateMsg: (%f, %u, %u) [%d]\n", 
                    fmsg.freq, fmsg.emid, fmsg.fcnt, (fmsg.fcnt > tcpServerTask->fCount));

                if (fmsg.fcnt > tcpServerTask->fCount) {
                    tcpServerTask->fCount = fmsg.fcnt;
                    tcpServerTask->freqCbFcn(fmsg.freq, fmsg.emid);
                }
            }
            else {
                tcpServerTask->validateWiFi();
                vTaskDelay(xThrottle); 
            }
        }

        Serial.println("Client disconnected");
        client.stop();
    }
}

void TcpServerTask::validateWiFi() {
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
    }
}
