#include "esp32/clk.h"
#include <Arduino.h>
#include <lwip/sockets.h>
#include <WiFi.h>
#include "Wire.h"
#include "CmdMsgs.h"
#include "ProjectSettings.h"


enum ServerMode { 
    INIT_INET = 1,
    SELECT_CLIENT,
    RECVFROM_CLIENT
};

#define SDA_PIN 21
#define SCL_PIN 22

// Server state
ServerMode serverMode;
int serverSockFd = -1;  // Datagram socket

// Reboot persistent frequency storage (in Slow Real Time Clock RAM)
RTC_NOINIT_ATTR uint32_t mHzBootPersist[3];
const uint32_t mHzRange[2] = {0, 2093000}; // 7 musical octaves

// I2C params
const uint8_t NUCLEO_ADDR = 0x37; // 0110111 (msb 7 bits)


void logBattLevel() {
    // Battery level on LBATT_PIN is thru 50/50 voltage divider
    double level = adc_to_voltage * 2.0 * analogRead(LBATT_PIN);
    Serial.printf("Battery: %0.2f V\n", level);
}

void logSystemStats() {
    Serial.printf("\nCPU frequency: %u MHz\n", (int)(esp_clk_cpu_freq() / 1e6));
    Serial.printf("FreeRTOS tick rate: %u kHz\n", portTICK_PERIOD_MS);
    Serial.printf("Main loop task priority: %u\n", uxTaskPriorityGet(NULL));
    Serial.printf("Total heap size: %u kb\n", ESP.getHeapSize() / 1024);
    Serial.printf("Free heap: %u kb\n", ESP.getFreeHeap() / 1024);
    Serial.printf("Lowest level of free heap since boot: %u kb\n", ESP.getMinFreeHeap() / 1024);
    Serial.printf("Largest block of heap that can be allocated at once: %u kb\n", ESP.getMaxAllocHeap() / 1024);
}

int inetBind(uint16_t port) {
    // Create a IPv4 UDP/datagram socket
    int sfd = socket(AF_INET , SOCK_DGRAM, 0);
    if (sfd == -1) {
        Serial.printf("socket ERRNO %d: %s\n", errno, strerror(errno));
        return -1;
    }

    // Server address struct (for udp://0.0.0.0:UDP_PORT)
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

    Serial.printf("Successful bind of UDP sockfd: %d to %s:%u (NBO: %s:%u)\n", 
        sfd, inet_ntoa(svaddr.sin_addr), port, inet_ntoa(svaddr.sin_addr), svaddr.sin_port);
    
    return sfd;
}

// Init WiFi
void inetInitWiFi() {
    if (!wifissid) Serial.printf("ERROR: WIFI SSID uninitialized (see ProjectSettings.h)");
    if (!wifipword) Serial.printf("ERROR: WIFI PWORD uninitialized (see ProjectSettings.h)");

    uint32_t elapsed = 0; 
    WiFi.begin(wifissid, wifipword);
    while (WiFi.status() != WL_CONNECTED) {
        // Serial.print("+");
        Serial.printf("+ : %u\n", WiFi.status());
        delay(1000);

        elapsed += 1;
        if (elapsed > 20) {
            Serial.println("terminal WiFi connect timeout, device reset initiated");
            // driverTask->getFrequency(mHzBootPersist); // Persist state before restart
            ESP.restart();
        }
    }
    Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString());
}

// Init UDP server sockets
void inetInitServer() {
    if (serverSockFd != -1 and close(serverSockFd) == -1)
        Serial.printf("close ERRNO %d: %s\n", errno, strerror(errno));
    
    serverSockFd = -1;
    uint32_t elapsed = 0;

    while (true) {
        serverSockFd = inetBind(UDP_PORT);
        if (serverSockFd != -1) break;
        Serial.print("+-");
        delay(1000);

        elapsed += 1;
        if (elapsed > 20) {
            Serial.println("terminal inetBind failure, device reset initiated");
            // driverTask->getFrequency(mHzBootPersist); // Persist state before restart
            ESP.restart();
        }
    }

    Serial.printf("Free heap after network init: %u kb\n", ESP.getFreeHeap() / 1024);
}

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) delay(100);

    Wire.begin(SDA_PIN, SCL_PIN, 100000U);
    serverMode = ServerMode::INIT_INET;
    logBattLevel();
    logSystemStats();
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

uint8_t i2cSetFrequency(struct EMDriverMsg &msg) {
    static uint8_t i2cbuffer[8];
    Serial.printf("i2cSetFrequency : [");
	for (int i=0, j=24; i < 4; ++i, j -= 8) {
	    i2cbuffer[i] = (uint8_t)(0xFF & (msg.freq_a >> j));
        Serial.printf("%u, ", i2cbuffer[i]);
    }
    Serial.printf("] : %u \n", msg.freq_a);

    Wire.beginTransmission(NUCLEO_ADDR);
    Wire.write(i2cbuffer, sizeof(uint32_t));
    uint8_t err = Wire.endTransmission();
    if (err) logI2CError(err);
    return err;
}

uint8_t i2cGetFrequency(struct EMDriverMsg &msg) {
    uint32_t freq = 0;
    Serial.printf("i2cGetFrequency : [");
    Wire.requestFrom(NUCLEO_ADDR, sizeof(uint32_t));
    while (Wire.available()) {
        uint8_t bt = Wire.read();
        freq = (freq << 8) | bt;
        Serial.printf(" %u,", bt);
    }
    Serial.printf("] : %u \n", freq);
    uint8_t err = Wire.endTransmission();
    if (err) logI2CError(err);

    msg.freq_a = freq;
    msg.freq_b = 1022;
    msg.freq_c = 1022;
    return err;
}

void handleMsg(char *msgbuffer, ssize_t nbytes, struct sockaddr *p_claddr, socklen_t claddrlen) {
    // Serial.printf("handleMsg from: %s:%d\n", inet_ntoa(claddr->sin_addr), claddr->sin_port);
    const static size_t msglen = sizeof(EMDriverMsg);
    static unsigned char sendbuffer[msglen];

    int ii = 0;
    while (ii + msglen <= nbytes) {
        EMDriverMsg msg;
        memcpy(&msg, &msgbuffer[ii], msglen);
        ii += msglen;

        Serial.printf("EMDriverMsg: (%u, %u, %d, %d, %d)\n", 
            msg.ackbit, msg.msgid, msg.freq_a, msg.freq_b, msg.freq_c);

        if (msg.freq_a != -1)
            i2cSetFrequency(msg);

        if (msg.ackbit) {
            i2cGetFrequency(msg);
            memcpy(sendbuffer, &msg, msglen);
            if (sendto(serverSockFd, sendbuffer, msglen, 0, p_claddr, claddrlen) != msglen)
                Serial.printf("sendto ERRNO %d: %s\n", errno, strerror(errno));
        }
    }
}

void loop() {
    static char msgbuffer[MAXMSGSZ];
    static struct sockaddr_in claddr;
    static struct timeval timeout;
    static fd_set readfds;

    switch(serverMode) {
    case ServerMode::INIT_INET: {
        inetInitWiFi();
        inetInitServer();
        serverMode = ServerMode::SELECT_CLIENT;
        break;
    }
    case ServerMode::SELECT_CLIENT: {
        timeout.tv_usec = 0;
        timeout.tv_sec = 2;

        FD_ZERO(&readfds);
        FD_SET(serverSockFd, &readfds);

        int ready = select(serverSockFd+1, &readfds, NULL, NULL, &timeout);
        if (ready == -1) Serial.printf("select ERRNO %d: %s\n", errno, strerror(errno));
        else if (ready > 0) serverMode = ServerMode::RECVFROM_CLIENT;
        else if (WiFi.status() != WL_CONNECTED) serverMode = ServerMode::INIT_INET;
        break;
    }
    case ServerMode::RECVFROM_CLIENT: {
        memset(&claddr, 0, sizeof(claddr));
        socklen_t claddrlen = sizeof(claddr);
        ssize_t nbytes = recvfrom(serverSockFd, msgbuffer, MAXMSGSZ,
            0, (struct sockaddr *)&claddr, &claddrlen);

        if (nbytes == -1) Serial.printf("recvfrom ERRNO %d: %s\n", errno, strerror(errno));
        else handleMsg(&msgbuffer[0], nbytes, (struct sockaddr *)&claddr, claddrlen);
        serverMode = ServerMode::SELECT_CLIENT;
        break;
    }
    default:
        Serial.printf("Invalid ServerMode: %d\n", serverMode);
    }
}

