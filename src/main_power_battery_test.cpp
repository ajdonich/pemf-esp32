// #include <vector>
#include <map>

#include "esp32/clk.h"
#include <Arduino.h>
#include "Wire.h"
#include <lwip/sockets.h>
#include <WiFi.h>

#include "CmdMsgs.h"
#include "ProjectSettings.h"

#define p3V3_PIN 35
#define p5V_PIN 36

typedef uint8_t i2c_test_func(uint16_t mapid, unsigned char *rspbuffer);

static const size_t MAXREQSZ = 128;
static const size_t MAXRSPSZ = 256;
static const size_t SZ8 = sizeof(int8_t);
static const size_t SZ16 = sizeof(int16_t);

static const uint8_t BGAUGE_ADDR = 0x55; // 1010101 (fixed i2c msb 7 bits)
static const uint16_t UDP_PORT_BATT = 4696;

enum ServerMode { 
    INIT_INET = 1,
    SELECT_CLIENT,
    RECVFROM_CLIENT
};

// Server state
ServerMode serverMode;
int serverSockFd = -1;  // Datagram socket


void logSystemStats() {
    Serial.printf("\nCPU frequency: %u MHz\n", (int)(esp_clk_cpu_freq() / 1e6));
    Serial.printf("FreeRTOS tick rate: %u kHz\n", portTICK_PERIOD_MS);
    Serial.printf("Main loop task priority: %u\n", uxTaskPriorityGet(NULL));
    Serial.printf("Total heap size: %u kb\n", ESP.getHeapSize() / 1024);
    Serial.printf("Free heap: %u kb\n", ESP.getFreeHeap() / 1024);
    Serial.printf("Lowest level of free heap since boot: %u kb\n", ESP.getMinFreeHeap() / 1024);
    Serial.printf("Largest block of heap that can be allocated at once: %u kb\n", ESP.getMaxAllocHeap() / 1024);
}

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) delay(100);
 
    serverMode = ServerMode::INIT_INET;
    logSystemStats();

    // Begin I2C standard-mode (100kHz)
    Wire.begin(SDA_PIN, SCL_PIN, 100000U);
}


int inetBind(uint16_t port) {
    // Create a IPv4 UDP/datagram socket
    int sfd = socket(AF_INET , SOCK_DGRAM, 0);
    if (sfd == -1) {
        Serial.printf("socket ERRNO %d: %s\n", errno, strerror(errno));
        return -1;
    }

    // Server address struct (for udp://0.0.0.0:UDP_PORT_BATT)
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
        serverSockFd = inetBind(UDP_PORT_BATT);
        if (serverSockFd != -1) break;
        Serial.print("+-");
        delay(1000);

        elapsed += 1;
        if (elapsed > 20) {
            Serial.println("terminal inetBind failure, device reset initiated");
            ESP.restart();
        }
    }

    Serial.printf("Free heap after network init: %u kb\n", ESP.getFreeHeap() / 1024);
}

std::map<uint8_t, const char*> i2cStdCmdMap {
    {0x00, "Control"},
    {0x02, "AtRate"},
    {0x04, "AtRateTimeToEmpty"},
    {0x06, "Temperature"},
    {0x08, "Voltage"},
    {0x0A, "BatteryStatus"},
    {0x0C, "Current"},
    {0x10, "RemainingCapacity"},
    {0x12, "FullChargeCapacity"},
    {0x14, "AverageCurrent"},
    {0x16, "TimeToEmpty"},
    {0x18, "TimeToFull"},
    {0x1A, "StandbyCurrent"},
    {0x1C, "StandbyTimeToEmpty"},
    {0x1E, "MaxLoadCurrent"},
    {0x20, "MaxLoadTimeToEmpty"},
    {0x22, "RawCoulombCount"},
    {0x24, "AveragePower"},
    {0x28, "InternalTemperature"},
    {0x2A, "CycleCount"},
    {0x2C, "RelativeStateOfCharge"},
    {0x2E, "StateOfHealth"},
    {0x30, "ChargeVoltage"},
    {0x32, "ChargeCurrent"},
    {0x34, "BTPDischargeSet"},
    {0x36, "BTPChargeSet"},
    {0x3A, "OperationStatus"},
    {0x3C, "DesignCapacity"},
    {0x3E, "ManufacturerAccessControl"},
    {0x3F, "Control/SubControl"}
};

std::map<uint16_t, const char*> ctrlSubCmdMap {
    {0x0000, "CONTROL_STATUS"},
    {0x0001, "DEVICE_NUMBER"},
    {0x0002, "FW_VERSION"},
    {0x0003, "HW_VERSION"},
    {0x0009, "BOARD_OFFSET"},
    {0x000A, "CC_OFFSET"},
    {0x000B, "CC_OFFSET_SAVE"},
    {0x000C, "OCV_CMD"},
    {0x000D, "BAT_INSERT"},
    {0x000E, "BAT_REMOVE"},
    {0x0013, "SET_SNOOZE"},
    {0x0014, "CLEAR_SNOOZE"},
    {0x0015, "SET_PROFILE_1"},
    {0x0016, "SET_PROFILE_2"},
    {0x0017, "SET_PROFILE_3"},
    {0x0018, "SET_PROFILE_4"},
    {0x0019, "SET_PROFILE_5"},
    {0x001A, "SET_PROFILE_6"},
    {0x002D, "CAL_TOGGLE"},
    {0x0030, "SEALED"},
    {0x0041, "RESET"},
    {0x0080, "EXIT_CAL"},
    {0x0081, "ENTER_CAL"},
    {0x0090, "ENTER_CFG_UPDATE"},
    {0x0091, "EXIT_CFG_UPDATE_REINIT"},
    {0x0092, "EXIT_CFG_UPDATE"},
    {0x0F00, "RETURN_TO_ROM"},
};

uint8_t i2cEnterFullAccess() {
    // Enter FULL ACCESS mode (required for access to RAM)
    Serial.println("Entering FULL ACCESS mode");
    Wire.beginTransmission(BGAUGE_ADDR);
    Wire.write(0x00);
    Wire.write(0xFF);
    Wire.write(0xFF);
    uint8_t err = Wire.endTransmission();
    if (err) return err;

    // Takes 2 identical write cmds 
    Wire.beginTransmission(BGAUGE_ADDR);
    Wire.write(0x00);
    Wire.write(0xFF);
    Wire.write(0xFF);
    return Wire.endTransmission();
}

uint8_t i2cEnterCfgUpdate(int nsecs=5) {
    // Enter CFGUPDATE with Command() ENTER_CGF_UPDATE
    Wire.beginTransmission(BGAUGE_ADDR);
    Wire.write(0x00);
    Wire.write(0x90);
    Wire.write(0x00);
    uint8_t err = Wire.endTransmission();
    if (err) return err;

    // Poll OperationStatus() until CFGUPDATE bit is set (should be at most 1 sec)
    int8_t isbitset = 0;
    Serial.print("Waiting for ENTER_CGF_UPDATE.");
    for (int x=0; x<nsecs; x++) {
        Wire.beginTransmission(BGAUGE_ADDR);
        Wire.write(0x3B);
        if (err = Wire.endTransmission(false)) {
            Wire.endTransmission();
            return err;
        }

        Wire.requestFrom(BGAUGE_ADDR, (size_t)1, true);
        while (Wire.available()) isbitset = Wire.read() & 0x4;
        if (isbitset) break;
        Serial.print(".");
        delay(1000); // 1s delay
    }
    
    Serial.println(isbitset ? "Complete" : "Failed");
    return isbitset ? 0 : 4; // "other error"
}

// Tries EXIT_CFG_UPDATE_REINIT first, if fails EXIT_CFG_UPDATE
uint8_t i2cExitCfgUpdate(uint16_t exitcmd=0x0091, int nsecs=5) {
    static uint16_t exitcmds[] = {0x0091, 0x0092};

    uint8_t err = 0;
    int8_t isbitset = 0x4;
    for(const uint16_t &xcmd : exitcmds) {
        // Exit CFGUPDATE with Command() 
        Wire.beginTransmission(BGAUGE_ADDR);
        Wire.write(0x00);
        Wire.write(xcmd & 0xFF);
        Wire.write(xcmd >> 8); 
        if (err = Wire.endTransmission()) 
            return err;

        // Poll OperationStatus() until CFGUPDATE bit is unset
        Serial.printf("Waiting for %s.", ctrlSubCmdMap[xcmd]);
        for (int x=0; x<nsecs; x++) {
            Wire.beginTransmission(BGAUGE_ADDR);
            Wire.write(0x3B);
            if (err = Wire.endTransmission(false)) {
                Wire.endTransmission();
                return err;
            }

            Wire.requestFrom(BGAUGE_ADDR, (size_t)1, true);
            while (Wire.available()) isbitset = Wire.read() & 0x4;
            if (!isbitset) break;
            Serial.print(".");
            delay(1000); // 1s delay
        }

        Serial.println(isbitset ? "Failed" : "Complete");
        if (!isbitset) return 0;
    }

    return 4; // I2C other error
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

uint8_t checksum8(ssize_t nbytes, const unsigned char *buffer) {
    uint32_t sum8 = 0;
    for (int i=0; i<nbytes; ++i)
        sum8 += (uint8_t)buffer[i];

    return (uint8_t)(0xFF - (sum8 & 0xFF));
}

bool isValidMic(ssize_t nbytes, char *msgbuffer) {
    uint8_t chksum = checksum8(nbytes-1, (const unsigned char *)msgbuffer);
    uint8_t mic = (uint8_t)msgbuffer[nbytes-1];
    return chksum == mic;
}

// Returns the chksum, blklen pair as a uint16_t. See BQ27220 manual, 
// section 2.30 MACDataSum() and 2.31 MACDataLen(). Expects memblock format:
//      [(0x3E..0x3F) (0x40..0x41) (0x42..0x5F)  (0x60)   (0x61) ]
//      [ (cmd addr)  (RAM @addr)  (other data) (chksum) (blklen)]
uint16_t calcChkLen(uint8_t memblock[36]) {
    uint8_t blklen = memblock[35];
    if (blklen != 36) Serial.printf("WARNING: atypical MACDataLen %u\n", blklen);

    uint32_t sum8 = 0;
    for (int i=0; i<(blklen-2); ++i)
        sum8 += memblock[i];

    uint8_t chksum = (uint8_t)(0xFF - (sum8 & 0xFF));
    Serial.printf("Calculated chksum-blklen: %.2X %.2X\n", chksum, blklen);
    return (chksum << 8) | blklen;
}

// Standard command (each is 2 byte read/response from "cmdid address")
uint8_t i2cStdCmd(uint8_t cmdid, size_t &ii, unsigned char *rspbuffer) {
    Wire.beginTransmission(BGAUGE_ADDR);
    Wire.write(cmdid);
    uint8_t err = Wire.endTransmission(false);
    if (err) {
        Wire.endTransmission();
        return err;
    }

    Serial.printf("%s (0x%.2X) :", i2cStdCmdMap[cmdid], cmdid);
    rspbuffer[ii] = cmdid; 
    rspbuffer[ii+1] = 2;
    ii += 2;

    Wire.requestFrom(BGAUGE_ADDR, SZ16, true);
    while (Wire.available()) {
        rspbuffer[ii] = Wire.read();
        Serial.printf(" %.2X", rspbuffer[ii]);
        ii += 1;
    }
    Serial.println();
    return 0;
}

uint8_t i2cCntlSubcmd(uint16_t subcmd, size_t &ii, unsigned char *rspbuffer) {
    Wire.beginTransmission(BGAUGE_ADDR);
    Wire.write(0x00);
    Wire.write(subcmd & 0xFF);
    Wire.write(subcmd >> 8);
    uint8_t err = Wire.endTransmission();
    if (err) return err;
    
    // Wait 2 seconds for existing RAM block to be written to MACData
    delay(2000);

    // Start read at ManufacturerAccessControl() (0x3E), expect the 2 subcmd
    // bytes followed by all 32 bytes of the MACData() block (0x40 thru 0x5F)
    Wire.beginTransmission(BGAUGE_ADDR);
    Wire.write(0x3E);
    if (err = Wire.endTransmission(false)) {
        Wire.endTransmission();
        return err;
    }

    Serial.printf("%s (0x%.2X) :", i2cStdCmdMap[0x3F], 0x3F);
    rspbuffer[ii] = 0x3F; 
    rspbuffer[ii+1] = 34;
    ii += 2;

    Wire.requestFrom(BGAUGE_ADDR, (size_t)34, true);
    while (Wire.available()) { 
        rspbuffer[ii] = Wire.read();
        Serial.printf(" %.2X", rspbuffer[ii]);
        ii += 1;
    }
    Serial.println();
    return 0;
}

uint8_t i2cRAMRead(uint16_t memaddr, size_t &ii, unsigned char *rspbuffer) {
    uint8_t err = 0;
    if (err = i2cEnterFullAccess())
        return err;

    // Write RAM address to ManufacturerAccessControl() 
    Wire.beginTransmission(BGAUGE_ADDR);
    Wire.write(0x3E);
    Wire.write(memaddr & 0xFF); 
    Wire.write(memaddr >> 8); 
    if (err = Wire.endTransmission()) 
        return err;
    
    // Wait 2 seconds for existing RAM block to be written to MACData
    delay(2000);

    // Read full 36 byte datablock 0x3E thru 0x61 
    Wire.beginTransmission(BGAUGE_ADDR);
    Wire.write(0x3E);
    if (err = Wire.endTransmission(false)) {
        Wire.endTransmission();
        return err;
    }

    Serial.printf("%s (0x%.2X) :", i2cStdCmdMap[0x3E], 0x3E);
    rspbuffer[ii] = 0x3E; 
    rspbuffer[ii+1] = 36;
    ii += 2;

    Wire.requestFrom(BGAUGE_ADDR, (size_t)36, true);
    while (Wire.available()) { 
        rspbuffer[ii] = Wire.read();
        Serial.printf(" %.2X", rspbuffer[ii]);
        ii += 1;
    }
    Serial.println();
    return 0;
}

uint8_t i2cRAMWrite(uint16_t memaddr, uint16_t dataval, uint16_t chklen) {
    uint8_t err = 0;
    if (err = i2cEnterFullAccess())
        return err;

    // Enter CFGUPDATE mode
    if (err = i2cEnterCfgUpdate()) 
        return err;

    // Write RAM address to ManufacturerAccessControl() 
    Wire.beginTransmission(BGAUGE_ADDR);
    Wire.write(0x3E);
    Wire.write(memaddr & 0xFF); 
    Wire.write(memaddr >> 8); 
    if (err = Wire.endTransmission()) 
        return err;
    
    // Wait 2 seconds for existing RAM block to be written to MACData
    delay(2000);

    // (Over-)write first two bytes of MACData block with new data value
    Wire.beginTransmission(BGAUGE_ADDR);
    Wire.write(0x40);
    Wire.write(dataval >> 8);
    Wire.write(dataval & 0xFF);
    if (err = Wire.endTransmission())
        return err;

    // (Over-)write MACDataSum-MACDataLen bytes with new checksum-blklen value
    Wire.beginTransmission(BGAUGE_ADDR);
    Wire.write(0x60);
    Wire.write(chklen >> 8);
    Wire.write(chklen & 0xFF);
    if (err = Wire.endTransmission())
        return err;

    // Exit CFGUPDATE mode (includes _REINIT)
    return i2cExitCfgUpdate();
}

void runStdCmds(char *msgbuffer, ssize_t nbytes, struct sockaddr *p_claddr, socklen_t claddrlen) {
    static unsigned char sendbuffer[MAXRSPSZ];

    size_t ii = 0;
    for (int jj = 1; jj < nbytes-1; ++jj) {
        uint8_t cmdid = (uint8_t)msgbuffer[jj];

        auto it = i2cStdCmdMap.find(cmdid);
        if (it == i2cStdCmdMap.end()) {
            Serial.printf("Invalid i2cStdCmd ID: %u\n", cmdid);
            continue;
        }

        uint8_t err = i2cStdCmd(cmdid, ii, sendbuffer);
        if (err) logI2CError(err);
    }

    if (ii == MAXRSPSZ) ii -= 1;
    sendbuffer[ii] = checksum8(ii, sendbuffer);
    ii += 1;

    Serial.printf("Sending %u bytes, MIC: %u\n", ii, sendbuffer[ii-1]);
    if (sendto(serverSockFd, sendbuffer, ii, 0, p_claddr, claddrlen) != ii)
        Serial.printf("sendto ERRNO %d: %s\n", errno, strerror(errno));
}

void runCtrlCmds(char *msgbuffer, ssize_t nbytes, struct sockaddr *p_claddr, socklen_t claddrlen) {
    static unsigned char sendbuffer[MAXRSPSZ];

    uint16_t ctrlcmd = 0;
    memcpy(&ctrlcmd, &msgbuffer[1], SZ16);
    auto it = ctrlSubCmdMap.find(ctrlcmd);
    if (it == ctrlSubCmdMap.end()) {
        Serial.printf("Invalid ctrlSubCmd ID: %u\n", ctrlcmd);
        return;
    }

    size_t ii = 0;
    uint8_t err = i2cCntlSubcmd(ctrlcmd, ii, sendbuffer);
    if (err) logI2CError(err);

    if (ii == MAXRSPSZ) ii -= 1;
    sendbuffer[ii] = checksum8(ii, sendbuffer);
    ii += 1;

    Serial.printf("Sending %u bytes, MIC: %u\n", ii, sendbuffer[ii-1]);
    if (sendto(serverSockFd, sendbuffer, ii, 0, p_claddr, claddrlen) != ii)
        Serial.printf("sendto ERRNO %d: %s\n", errno, strerror(errno));
}


void runMemoryCmd(char *msgbuffer, ssize_t nbytes, struct sockaddr *p_claddr, socklen_t claddrlen) {
    static unsigned char sendbuffer[MAXRSPSZ];

    size_t ii = 0;
    uint16_t memaddr = 0;
    memcpy(&memaddr, &msgbuffer[1], SZ16);
    uint8_t err = i2cRAMRead(memaddr, ii, sendbuffer);
    if (err) logI2CError(err);

    sendbuffer[ii] = checksum8(ii, sendbuffer);  // MIC
    ii += 1;

    Serial.printf("Sending %u bytes, MIC: %u\n", ii, sendbuffer[ii-1]);
    if (sendto(serverSockFd, sendbuffer, ii, 0, p_claddr, claddrlen) != ii)
        Serial.printf("sendto ERRNO %d: %s\n", errno, strerror(errno));
}

void runMemoryUpdate(char *msgbuffer, ssize_t nbytes, struct sockaddr *p_claddr, socklen_t claddrlen) {
    static unsigned char sendbuffer[MAXRSPSZ];

    uint16_t memaddr(0), dataval(0);
    memcpy(&memaddr, &msgbuffer[1], SZ16);
    memcpy(&dataval, &msgbuffer[3], SZ16);

    // Read-in
    uint8_t err = 0;
    size_t ii(0), jj(0);
    uint8_t memblock[38]; // 36 plus 2 header bytes
    if (err = i2cRAMRead(memaddr, jj, memblock)) 
        logI2CError(err);

    // Over-write dataval and chklen
    memcpy(&memblock[4], &dataval, SZ16);
    uint16_t chklen = calcChkLen(&memblock[2]);
    if (err = i2cRAMWrite(memaddr, dataval, chklen))
        logI2CError(err);

    // (Re-)read to verify
    if (err = i2cRAMRead(memaddr, ii, sendbuffer)) 
        logI2CError(err);

    sendbuffer[ii] = checksum8(ii, sendbuffer); // MIC
    ii += 1;

    Serial.printf("Sending %u bytes, MIC: %u\n", ii, sendbuffer[ii-1]);
    if (sendto(serverSockFd, sendbuffer, ii, 0, p_claddr, claddrlen) != ii)
        Serial.printf("sendto ERRNO %d: %s\n", errno, strerror(errno));
}

inline uint32_t telapsed() {
    static uint64_t toffset = micros();
    return (uint32_t)(micros() - toffset);
}

void loop() {
    static char msgbuffer[MAXMSGSZ];
    static struct sockaddr_in claddr;
    static struct timeval timeout;
    static fd_set readfds;

    static uint32_t tlastreport = telapsed(); 
    if (telapsed() - tlastreport > 5e5) {
        static double vdivide = (9.83e3+9.88e3)/9.88e3;
        double v33 = adc_to_voltage * analogRead(p3V3_PIN);
        double v5 = adc_to_voltage * analogRead(p5V_PIN);
        Serial.printf("3V3: %0.2f, 5V %0.2f (%0.2f)\n", v33, v5*vdivide, v5);
        
        // uint32_t v33 = analogReadMilliVolts(p3V3_PIN);
        // uint32_t v5 = analogReadMilliVolts(p5V_PIN);
        // Serial.printf("3V3: %0.3f, 5V %0.3f (%0.3f)\n", v33 * 1e-3, v5 * 1e-3 * vdivide, v5 * 1e-3);

        tlastreport = telapsed();
    }

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

        //Serial.printf("RECVFROM_CLIENT: %s:%d\n", inet_ntoa(claddr.sin_addr), claddr.sin_port);
        if (nbytes == -1) Serial.printf("recvfrom ERRNO %d: %s\n", errno, strerror(errno));
        else if (!isValidMic(nbytes, msgbuffer)) Serial.println("RECVFROM_CLIENT ERR: invalid MIC");
        else {
            GaugeCmdType mtype = (GaugeCmdType)((int)msgbuffer[0]);
            if (mtype == GaugeCmdType::STDCMD)
                runStdCmds(msgbuffer, nbytes, (struct sockaddr *)&claddr, claddrlen);
            if ( mtype == GaugeCmdType::CTRLCMD)
                runCtrlCmds(msgbuffer, nbytes, (struct sockaddr *)&claddr, claddrlen);
            else if (mtype == GaugeCmdType::RAMREAD) 
                runMemoryCmd(msgbuffer, nbytes, (struct sockaddr *)&claddr, claddrlen);
            else if (mtype == GaugeCmdType::RAMWRITE) 
                runMemoryUpdate(msgbuffer, nbytes, (struct sockaddr *)&claddr, claddrlen);
        }
        serverMode = ServerMode::SELECT_CLIENT;
        break;
    }
    default:
        Serial.printf("Invalid ServerMode: %d\n", serverMode);
    }
}

// uint8_t i2cRAMDesignCapacity() {
//     // RAM data request: write DesignCapacity RAM address (0x929F) to ManufacturerAccessControl (0x3E)
//     Wire.beginTransmission(BGAUGE_ADDR);
//     Wire.write(0x3E);
//     Wire.write(0x9F);
//     Wire.write(0x92);
//     uint8_t err = Wire.endTransmission();
//     if (err) return err;
//     else delay(2000); // 2s delay

//     // Read data response from MACData (0x40)
//     Wire.beginTransmission(BGAUGE_ADDR);
//     Wire.write(0x40);
//     if (err = Wire.endTransmission(false)) {
//         Wire.endTransmission();
//         return err;
//     }

//     Wire.requestFrom(BGAUGE_ADDR, (size_t)32, true);
//     while (Wire.available()) { 
//         Serial.printf("%.2X ", Wire.read());
//     }
//     Serial.println();
//     return 0;
// }

// std::map<uint16_t, i2c_test_func*> i2cfcnmap {
//     {29, i2cCFGUPDATETest},
//     // {30, i2cWriteRAMDesignCapacity},
//     // {31, i2cSpecial},
//     {32, i2cControl}
// };


// uint8_t i2cTemperature() { Serial.println("Temperature: "); return 0; }
// uint8_t i2cVoltage() { Serial.println("Voltage: "); return 0; }
// uint8_t i2cBatteryStatus() { Serial.println("BatteryStatus: "); return 0; }
// uint8_t i2cCurrent() { Serial.println("Current: "); return 0; }
// uint8_t i2cRemainingCapacity() { Serial.println("RemainingCapacity: "); return 0; }
// uint8_t i2cFullChargeCapacity() { Serial.println("FullChargeCapacity: "); return 0; }
// uint8_t i2cAverageCurrent() { Serial.println("AverageCurrent: "); return 0; }
// uint8_t i2cTimeToEmpty() { Serial.println("TimeToEmpty: "); return 0; }
// uint8_t i2cTimeToFull() { Serial.println("TimeToFull: "); return 0; }
// uint8_t i2cStandbyCurrent() { Serial.println("StandbyCurrent: "); return 0; }
// uint8_t i2cStandbyTimeToEmpty() { Serial.println("StandbyTimeToEmpty: "); return 0; }
// uint8_t i2cMaxLoadCurrent() { Serial.println("MaxLoadCurrent: "); return 0; }
// uint8_t i2cMaxLoadTimeToEmpty() { Serial.println("MaxLoadTimeToEmpty: "); return 0; }
// uint8_t i2cRawCoulombCount() { Serial.println("RawCoulombCount: "); return 0; }
// uint8_t i2cAveragePower() { Serial.println("AveragePower: "); return 0; }
// uint8_t i2cCycleCount() { Serial.println("CycleCount: "); return 0; }
// uint8_t i2cRelativeStateOfCharge() { Serial.println("RelativeStateOfCharge: "); return 0; }
// uint8_t i2cStateOfHealth() { Serial.println("StateOfHealth: "); return 0; }
// uint8_t i2cChargeVoltage() { Serial.println("ChargeVoltage: "); return 0; }
// uint8_t i2cChargeCurrent() { Serial.println("ChargeCurrent: "); return 0; }
// uint8_t i2cOperationStatus() { Serial.println("OperationStatus: "); return 0; }
// uint8_t i2cDesignCapacity() { Serial.println("DesignCapacity: "); return 0; }


    // // Read MACDataSum()
    // Wire.beginTransmission(BGAUGE_ADDR);
    // Wire.write(0x60);
    // if (err = Wire.endTransmission(false)) {
    //     Wire.endTransmission();
    //     return err;
    // }

    // Serial.print("MACDataSum:");
    // Wire.requestFrom(BGAUGE_ADDR, (size_t)1, true);
    // while (Wire.available()) { 
    //     Serial.printf(" %.2X", Wire.read());
    // }
    // Serial.println();

    // // Read MACDataLen()
    // Wire.beginTransmission(BGAUGE_ADDR);
    // Wire.write(0x61);
    // if (err = Wire.endTransmission(false)) {
    //     Wire.endTransmission();
    //     return err;
    // }

    // Serial.print("MACDataLen:");
    // Wire.requestFrom(BGAUGE_ADDR, (size_t)1, true);
    // while (Wire.available()) { 
    //     Serial.printf(" %.2X", Wire.read());
    // }
    // Serial.println();

    // // Write (again) RAM DesignCapacity() to ManufacturerAccessControl()
    // Wire.beginTransmission(BGAUGE_ADDR);
    // Wire.write(0x3E);
    // Wire.write(0x9F);
    // Wire.write(0x92);
    // if (err = Wire.endTransmission()) return err;
    // else delay(2000); // 2s delay

// uint8_t i2cWriteRAMDesignCapacity(uint16_t mapid, unsigned char *rspbuffer) {
//     uint8_t err = 0;
//     if (err = i2cEnterFullAccess()) {
//         Serial.println("Enter FULL ACCESS mode failed");
//         return err;
//     }

//     // Enter CFGUPDATE mode
//     if (err = i2cEnterCfgUpdate()) return err;

//     // Write Profile1 DesignCapacity RAM address to ManufacturerAccessControl()
//     // and wait 2 seconds for existing RAM block to be written to MACData
//     Wire.beginTransmission(BGAUGE_ADDR);
//     Wire.write(0x3E);
//     Wire.write(0x9F);
//     Wire.write(0x92);
//     if (err = Wire.endTransmission()) return err;
//     else delay(2000); // 2s delay

//     // (Over-)write first two bytes of MACData block with new DesignCapacity (i.e. w/980 mAh (0x03d4))
//     Wire.beginTransmission(BGAUGE_ADDR);
//     Wire.write(0x40);
//     Wire.write(0x03);
//     Wire.write(0xd4);
//     if (err = Wire.endTransmission())
//         return err;

//     // (Over-)write the MACDataSum byte with new checksum (i.e. 0x8d, I precomputed this for 980 mAh)
//     Wire.beginTransmission(BGAUGE_ADDR);
//     Wire.write(0x60);
//     Wire.write(0x8d);
//     if (err = Wire.endTransmission())
//         return err;

//     // (Re-)write the block length (i.e. 0x24) to MACDataLen to trigger the actual RAM commit  
//     Wire.beginTransmission(BGAUGE_ADDR);
//     Wire.write(0x61);
//     Wire.write(0x24);
//     if (err = Wire.endTransmission())
//         return err;

//     // Exit CFGUPDATE mode (includes _REINIT)
//     if (err = i2cExitCfgUpdate()) return err;
//     memcpy(&rspbuffer[0], &mapid, (size_t)2);
//     memset(&rspbuffer[2], 0, (size_t)2);
//     return 0;
// }

// uint8_t i2cCFGUPDATETest(uint16_t mapid, unsigned char *rspbuffer) {
//     uint8_t err = 0;
//     if (err = i2cEnterFullAccess()) {
//         Serial.println("Enter FULL ACCESS mode failed");
//         return err;
//     }

//     if (err = i2cEnterCfgUpdate()) {
//         Serial.println("CFGUPDATE set failed");
//         return err;
//     }
//     Serial.println("CFGUPDATE set succeeded");

//     if (err = i2cExitCfgUpdate()) {
//         Serial.println("CFGUPDATE unset failed");
//         return err;
//     }
//     Serial.println("CFGUPDATE unset succeeded");

//     memcpy(&rspbuffer[0], &mapid, (size_t)2);
//     return 0;
// }

// uint8_t cntlSubcommand(uint16_t subcmd) {
//     Wire.beginTransmission(BGAUGE_ADDR);
//     Wire.write(0x00);
//     Wire.write(subcmd & 0xFF);
//     Wire.write(subcmd >> 8);
//     uint8_t err = Wire.endTransmission();
//     if (err) return err;
//     else delay(2000); // 2s delay

//     // Start read at ManufacturerAccessControl() (0x3E), expect the 2 subcmd
//     // bytes followed by all 32 bytes of the MACData() block (0x40 thru 0x5F)
//     Wire.beginTransmission(BGAUGE_ADDR);
//     Wire.write(0x3E);
//     if (err = Wire.endTransmission(false)) {
//         Wire.endTransmission();
//         return err;
//     }

//     Serial.printf("%s:", ctrlSubCmdMap[subcmd]);
//     Wire.requestFrom(BGAUGE_ADDR, (size_t)34, true);
//     while (Wire.available()) { 
//         Serial.printf(" %.2X", Wire.read());
//     }
//     Serial.println();
//     return 0;
// }

// uint8_t i2cControl(uint16_t mapid, unsigned char *rspbuffer) {
//     uint16_t subcommands[] = {0x0000, 0x0001, 0x0002};
//     for (int i=0; i<3; ++i) {
//         uint8_t err = cntlSubcommand(subcommands[i]);
//         if (err) return err;
//     }

//     Wire.beginTransmission(BGAUGE_ADDR);
//     Wire.write(0x00);
//     uint8_t err = Wire.endTransmission(false);
//     if (err) {
//         Wire.endTransmission();
//         return err;
//     }

//     int16_t ibt(0), rval(0);
//     Wire.requestFrom(BGAUGE_ADDR, (size_t)2, true);
//     while (Wire.available()) {
//         rval |= (Wire.read() << (ibt*8));
//         ibt += 1;
//     }
//     Serial.printf("Control: %i\n", rval);
//     memcpy(&rspbuffer[0], &mapid, (size_t)2);
//     memcpy(&rspbuffer[2], &rval, (size_t)2);
//     return 0;
// }
