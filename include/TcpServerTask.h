#include "CmdMsgs.h"

class WiFiClient;

class TcpServerTask {
public:
    TcpServerTask(freq_update_cb freqCbFcn_a=nullptr);
    ~TcpServerTask();

    void run();

private:
    freq_update_cb freqCbFcn;
    TaskHandle_t tcpServerTaskHandle;
    uint32_t fCount;

    static TickType_t xThrottle;
    static WiFiClient awaitConnection();
    static void tcpServerTaskLoop( void *pvParameters );
    void validateWiFi();
};
