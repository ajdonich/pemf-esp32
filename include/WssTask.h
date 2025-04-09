#include "esp_websocket_client.h"

typedef void (*freq_callback_t)(double);

struct QueueMessage {
    const char *data;
    size_t len;
    
    QueueMessage();
    QueueMessage(size_t alloc_len); // Allocates (empty) data on heap
    QueueMessage(uint8_t *data_a, size_t len_a); // Allocates data on heap
    ~QueueMessage(); // WARNING: does not free data. User must call free explicitly

    // WARNING: does not protect against buffer overwrite
    void append(const void *data_a, size_t len_a);
    void free();
};

class WssTask {
public:
    WssTask(freq_callback_t freqCbFcn_a=nullptr);
    ~WssTask();

    void run();
    void writeMessage(QueueMessage msg, bool toFront=false);

private:
    esp_websocket_client_handle_t wssclient;
    TaskHandle_t wssTaskHandle;
    QueueHandle_t taskMsgQueue;
    uint32_t msgSentCount;
    freq_callback_t freqCbFcn;

    static void wss_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
    static void wssTaskLoop( void *pvParameters );

    void wss_connect();
    void wifi_connect();
};
