#include <Arduino.h>
#include <WiFi.h>
#include "WssTask.h"
#include "ProjectSettings.h"

QueueMessage::QueueMessage() : data(nullptr), len(0) {}
QueueMessage::QueueMessage(size_t alloc_len) : len(0) {
    data = (const char*)(new uint8_t[alloc_len]); 
}

QueueMessage::QueueMessage(uint8_t *data_a, size_t len_a) : len(len_a) {
    void *buffer = new uint8_t[len_a];
    memcpy(buffer, (const void*)data_a, len_a);
    data = (const char*)buffer; 
}

QueueMessage::~QueueMessage() {}

void QueueMessage::free() { 
    if (data) delete[] data;
    data = nullptr;
    len = 0;
}

void QueueMessage::append(const void *data_a, size_t len_a) {
    memcpy((void*)&data[len], data_a, len_a);
    len += len_a;
}

WssTask::WssTask(freq_callback_t freqCbFcn_a) 
    : wssclient(nullptr), wssTaskHandle(nullptr), 
    taskMsgQueue(nullptr), msgSentCount(0), 
    freqCbFcn(freqCbFcn_a) {}

WssTask::~WssTask() {
    if (wssclient) {
        esp_websocket_client_stop(wssclient);
        esp_websocket_client_destroy(wssclient);
        wssclient = nullptr;
    }

    if (wssTaskHandle) {
        vTaskDelete(wssTaskHandle);
        wssTaskHandle = nullptr;
    }

    WiFi.disconnect();
}

void WssTask::run() {
    while (true) {
        if(taskMsgQueue = xQueueCreate(QUEUE_SIZE, sizeof( QueueMessage ))) break;
        Serial.println("ERROR: failure in xQueueCreate, unable to create MsgQueue");
        delay(1000);
    }

    while (true) {
        if (xTaskCreate(wssTaskLoop, "wss-task", WSS_STACK_DEPTH, (void*)this, 
            tskIDLE_PRIORITY, &wssTaskHandle ) == pdPASS) break;

        Serial.println("ERROR: failure in xTaskCreate, unable to create WssTask");
        delay(1000);
    }
}

// Queue msg for tranmission on WSS in FIFO order ASAP
void WssTask::writeMessage(QueueMessage msg, bool toFront) {
    static int dropCnt = 0;

    BaseType_t xResult = toFront
        ? xQueueSendToFront(taskMsgQueue, (void *)&msg, (TickType_t)0)
        : xQueueSend(taskMsgQueue, (void *)&msg, (TickType_t)0);

    if (xResult == errQUEUE_FULL) {
        if (dropCnt % 5000 == 0) Serial.println("taskMsgQueue still full, data is being dropped");
        dropCnt += 1;
        msg.free();
    }
}

void WssTask::wss_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {    
    switch (event_id) {
    case WEBSOCKET_EVENT_ERROR: 
        Serial.println("WEBSOCKET_EVENT_ERROR");
        break;

    case WEBSOCKET_EVENT_CONNECTED: 
        Serial.printf("WEBSOCKET_EVENT_CONNECTED: ws://%s:%d\n", WSS_HOST, WSS_PORT);
        // ndisconnect = 0;
        break;
        
    case WEBSOCKET_EVENT_DISCONNECTED: 
        Serial.println("WEBSOCKET_EVENT_DISCONNECTED");
        break;

    case WEBSOCKET_EVENT_DATA: {
        WssTask *wssTask = (WssTask*)handler_args;
        esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
        std::string sdata((char *)data->data_ptr, data->data_len);
        
        switch (data->op_code) {
        case 1:
            wssTask->freqCbFcn(atof(sdata.c_str()));
            // Serial.printf("WEBSOCKET_EVENT_DATA: %s\n", sdata.c_str());
            break;
        
        case 9: /*Serial.println("WEBSOCKET_EVENT_DATA: (PING)");*/ break;
        case 10: /*Serial.println("WEBSOCKET_EVENT_DATA: (PONG)");*/ break;
        default: Serial.printf("WEBSOCKET_EVENT_DATA: opcode=%d\n");
        }
        break;
    }
    default: Serial.printf("Unrecognized WEBSOCKET_EVENT: %d\n", event_id);  
    }
}

// Loop will long block on the queue for message arrival
void WssTask::wssTaskLoop( void *pvParameters ) {
    Serial.println("entered wss-task loop");
    TickType_t xDelay = 10000 / portTICK_PERIOD_MS;
    TickType_t xThrottle = 250 / portTICK_PERIOD_MS;
    WssTask *wssTask = (WssTask*)pvParameters;

    while (true) {
        wssTask->wifi_connect();
        wssTask->wss_connect();
        QueueMessage msg;

        // Will long block on the queue 10s or until message shows up
        if ( xQueueReceive(wssTask->taskMsgQueue, &msg, xDelay) == pdPASS ) {
            if ( esp_websocket_client_send_bin(wssTask->wssclient, msg.data, msg.len, portMAX_DELAY) == -1 ) {
                wssTask->writeMessage(msg, true); // Write msg back onto queue if send error
                continue;
            }
            wssTask->msgSentCount += 1;
            // Serial.print(wssTask->msgSentCount);
            // Serial.print("\r");

            static uint32_t moffset = 297;
            static uint32_t toffset = micros();
            static uint32_t lastreport = (micros() - toffset);
            if (micros() - lastreport > 5e6) {
                uint32_t elapsed = (uint32_t)(micros() - toffset) * 1e-6;
                UBaseType_t sz = QUEUE_SIZE - uxQueueSpacesAvailable(wssTask->taskMsgQueue);
                uint32_t nheap = ESP.getFreeHeap() / 1024;
                
                Serial.printf("%us, %u msgs queued, free heap: %ukb", elapsed, sz, nheap);
                if (sz > 0) Serial.printf(" (%.2f kb/msg)\n", (double)(moffset - nheap) / sz);
                else { Serial.println(); moffset = nheap; }
                
                lastreport = micros();
            }
        }

        msg.free();
        // vTaskDelay(xThrottle);
    }
}

void WssTask::wss_connect() {
    static esp_websocket_client_config_t websocket_cfg = { .host = WSS_HOST, .port = WSS_PORT };

    if (wssclient && !esp_websocket_client_is_connected(wssclient)) {
        esp_websocket_client_stop(wssclient);
        esp_websocket_client_destroy(wssclient);
        wssclient = nullptr;
    }

    if (!esp_websocket_client_is_connected(wssclient)) {
        wssclient = esp_websocket_client_init(&websocket_cfg);
        esp_websocket_register_events(wssclient, WEBSOCKET_EVENT_ANY, wss_event_handler, (void*)this);
        esp_websocket_client_start(wssclient);
        msgSentCount = 0;

        while (!esp_websocket_client_is_connected(wssclient)) {
            Serial.print(".");
            delay(1000);
        }
    }
}

void WssTask::wifi_connect() {
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