#include "esp32/clk.h"
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "FreqUpdateMsg.h"
// #include "TcpServerTask.h"
#include "EMDriverTask.h"
#include "ProjectSettings.h"

#include <lwip/sockets.h>
// #include <lwip/netdb.h>


enum ServerMode { 
    ACCEPT_CLIENT = 1,
    READ_CLIENT
};

// Server state
ServerMode serverMode;
int serverSockFd = -1; // Passive listen socket
int clientSockFd = -1; // Client socket

// EM driver
EMDriverTask *driverTask = nullptr;

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

int inetListen(uint16_t port, int backlog=3) {
    // Create a IPv4 TCP/steaming socket
    int sfd = socket(AF_INET , SOCK_STREAM, 0);
    if (sfd == -1) {
        Serial.printf("socket ERRNO %d: %s\n", errno, strerror(errno));
        return -1;
    }
    
    // Set reuseaddr
    int optval = 1;
    if (setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1) {
        Serial.printf("setsockopt SO_REUSEADDR ERRNO %d: %s\n", errno, strerror(errno));
        close(sfd);
        return -1;
    }

    // Server address struct (for tcp://0.0.0.0:TCP_PORT)
    struct sockaddr_in svaddr;
    memset(&svaddr, 0, sizeof(svaddr));
    svaddr.sin_family = AF_INET;         // IPv4
    svaddr.sin_port = htons(port);       // Port in network byte order
    svaddr.sin_addr.s_addr = INADDR_ANY; // Wildcard address: 0.0.0.0

    // Bind server address to socket
    if(bind(sfd, (struct sockaddr *)&svaddr, sizeof(svaddr)) == -1) {
        Serial.printf("bind ERRNO %d: %s\n", errno, strerror(errno));
        close(sfd);
        return -1;
    }

    // Mark socket as passive
    if(listen(sfd , backlog) == -1) {
        Serial.printf("listen ERRNO %d: %s\n", errno, strerror(errno));
        close(sfd);
        return -1;
    }

    return sfd;
}

// Init WiFi
void inetInitWiFi() {
    if (!wifissid) Serial.printf("ERROR: WIFI SSID uninitialized (see ProjectSettings.h)");
    if (!wifipword) Serial.printf("ERROR: WIFI PWORD uninitialized (see ProjectSettings.h)");

    uint32_t elapsed = 0; 
    WiFi.begin(wifissid, wifipword);
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

// Init listen server
void inetInitServer() {
    uint32_t elapsed = 0;
    while (true) {
        serverSockFd = inetListen(TCP_PORT);
        if (serverSockFd != -1) break;
        Serial.print("+-");
        delay(1000);

        elapsed += 1;
        if (elapsed > 20) {
            Serial.println("terminal inetListen failure, device reset initiated");
            ESP.restart();
        }
    }

    Serial.printf("Server listening on: tcp://%s:%d\n", 
        WiFi.localIP().toString(), TCP_PORT);
}

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) delay(100);

    logBattLevel();
    logSystemStats();
    inetInitWiFi();
    inetInitServer();
    Serial.printf("Free heap after network init: %u kb\n", ESP.getFreeHeap() / 1024);

    // TaskHandle_t thisTask = xTaskGetCurrentTaskHandle();
    Serial.printf("Main loop task priority: %u\n", uxTaskPriorityGet(NULL));

    serverMode = ServerMode::ACCEPT_CLIENT;
    driverTask = new EMDriverTask();
    driverTask->run();
}

const uint16_t MSGSZ = sizeof(FreqUpdateMsg);
char msgBuffer[MSGSZ];

void loop() {
    switch(serverMode) {
        case ServerMode::ACCEPT_CLIENT: {
            Serial.println("ServerMode::ACCEPT_CLIENT");
            clientSockFd = accept(serverSockFd, NULL, NULL);
            if (clientSockFd == -1) {
                Serial.printf("accept ERRNO %d: %s\n", errno, strerror(errno));
                break;
            }

            Serial.printf("Accepted connection: %d\n", clientSockFd);
            serverMode = ServerMode::READ_CLIENT;
            break;
        }
        case ServerMode::READ_CLIENT: {
            if (read(clientSockFd, msgBuffer, MSGSZ) == -1) {
                Serial.printf("read ERRNO %d: %s\n", errno, strerror(errno));
                close(clientSockFd);
                clientSockFd = -1;

                serverMode = ServerMode::ACCEPT_CLIENT;
                break;
            }

            FreqUpdateMsg fmsg;
            memcpy(&fmsg, msgBuffer, MSGSZ);
            driverTask->setFrequency(TAU*fmsg.freq, fmsg.emid-1);
            Serial.printf("FreqUpdateMsg: (%f, %u, %u)\n", 
                fmsg.freq, fmsg.emid, fmsg.fcnt);
            break;
        }
        default:
            Serial.printf("Invalid ServerMode: %d\n", serverMode);
            break;
    }
}
