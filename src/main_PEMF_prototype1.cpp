#include "esp32/clk.h"
#include <Arduino.h>
#include <lwip/sockets.h>
#include <WiFi.h>
#include "CmdMsgs.h"
#include "EMDriverTask.h"
#include "ProjectSettings.h"


enum ServerMode { 
    INIT_INET = 1,
    SELECT_CLIENT,
    RECVFROM_CLIENT
};

// Server state
ServerMode serverMode;
int serverSockFd = -1;  // Datagram socket

// EM driver
EMDriverTask *driverTask = nullptr;

// Reboot persistent frequency storage (in Slow Real Time Clock RAM)
RTC_NOINIT_ATTR uint32_t mHzBootPersist[3];
const uint32_t mHzRange[2] = {0, 2093000}; // 7 musical octaves


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
            driverTask->getFrequency(mHzBootPersist); // Persist state before restart
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
            driverTask->getFrequency(mHzBootPersist); // Persist state before restart
            ESP.restart();
        }
    }

    Serial.printf("Free heap after network init: %u kb\n", ESP.getFreeHeap() / 1024);
}

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) delay(100);

    serverMode = ServerMode::INIT_INET;
    driverTask = new EMDriverTask(1022, 1022, 1022);
    driverTask->run();

    // Note: due to _NOINIT, mHzBootPersist is garbage on power up (but will
    // persist through an ESP.restart), thus first validate against mHzRange
    if (mHzRange[0] <= mHzBootPersist[0] && mHzBootPersist[0] <= mHzRange[1]) {
        driverTask->setFrequency(mHzBootPersist);
        Serial.printf("\nsetup to mHzBootPersist: (%u, %u, %u)\n", 
            mHzBootPersist[0], mHzBootPersist[1], mHzBootPersist[2]);
    }

    logBattLevel();
    logSystemStats();
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
            driverTask->setFrequency(msg);

        if (msg.ackbit) {
            driverTask->getFrequency(msg);
            memcpy(sendbuffer, &msg, msglen);
            if (sendto(serverSockFd, sendbuffer, msglen, 0, p_claddr, claddrlen) != msglen)
                Serial.printf("sendto ERRNO %d: %s\n", errno, strerror(errno));
        }
    }
}

// TEMPORARY:
// uint64_t hopinterval = (uint64_t)(6e7 * 5.0); // 5 min
// uint64_t tminit = 0;
// uint32_t mHz[3] = {0,0,0};
// uint32_t scale[133] = {
//     1022,1083,1147,1215,1288,1364,1445,1531,1622,1719,1821,1929,2044,2165,2294,2431,
//     2575,2728,2891,3062,3245,3437,3642,3858,4088,4331,4589,4861,5150,5457,5781,6125,
//     6489,6875,7284,7717,8176,8662,9177,9723,10301,10913,11562,12250,12978,13750,14568,
//     15434,16352,17324,18354,19445,20602,21827,23125,24500,25957,27500,29135,30868,32703,
//     34648,36708,38891,41203,43654,46249,48999,51913,55000,58270,61735,65406,69296,73416,
//     77782,82407,87307,92499,97999,103826,110000,116541,123471,130813,138591,146832,155563,
//     164814,174614,184997,195998,207652,220000,233082,246942,261626,277183,293665,311127,
//     329628,349228,369994,391995,415305,440000,466164,493883,523251,554365,587330,622254,
//     659255,698456,739989,783991,830609,880000,932328,987767,1046502,1108731,1174659,
//     1244508,1318510,1396913,1479978,1567982,1661219,1760000,1864655,1975533,2093005};
// END TEMPORARY

void loop() {
    // TEMPORARY: 
    // if ((micros() - tminit) >= hopinterval) {
    //     mHz[2] = scale[(int)random(133)];
    //     Serial.printf("setFrequency: %.2f  Hz\n", (double)mHz[2] / 1e3);
    //     driverTask->setFrequency(mHz);
    //     tminit = micros();
    // }
    // return; 
    // END TEMPORARY: to run w/no network connectivity and hardcoded init frequencies 

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

