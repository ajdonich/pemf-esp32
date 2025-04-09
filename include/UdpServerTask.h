
#include <WiFiUdp.h>

typedef void (*freq_callback_t)(double freq, uint32_t emid);

struct UdpFreqMsg {
    double freq;
    uint32_t emid; 
    uint32_t fcnt;
};

class UdpServerTask {
public:
    UdpServerTask(freq_callback_t freqCbFcn_a=nullptr);
    ~UdpServerTask();

    void run();

private:
    freq_callback_t freqCbFcn;
    TaskHandle_t udpServerTaskHandle;
    WiFiUDP udpServer;
    uint32_t fCount;

    static void udpServerTaskLoop( void *pvParameters );
    void serve();
};
