#ifndef PROJECT_SETTINGS_H
#define PROJECT_SETTINGS_H

// Serial
#define BAUD_RATE 9600

// GPIO Pins 
#define RDY_PIN 5
#define LDAC_PIN 18
#define SDA_PIN 21
#define SCL_PIN 22
#define SIGA_PIN 34
#define LBATT_PIN 35
#define OPTA_PIN 39

// Consts
#define TAU 6.2831853071795864769252867665590
#define kTAU .0062831853071795864769252867665590

#define tskMID_PRIORITY  ( ( UBaseType_t ) 1U )  // Arduino main loop (network)
#define tskHIGH_PRIORITY ( ( UBaseType_t ) 2U )  // EM driver loop 

static const float adc_to_voltage = 3.3 / 4095.0;
static const float voltage_to_dac = 4095.0 / 3.3;

// Wifi params [init here, see env WIFISSID/WIFIPWORD]
static const char* wifissid  = nullptr;
static const char* wifipword = nullptr; 

// Wss server (external EspSampleServer.ipynb)
static const char* WSS_HOST = "192.168.0.17";
static const uint16_t WSS_PORT = 8765;

// UDP listen port (ESP32 server)
static const uint16_t UDP_PORT = 4645;
static const uint16_t TCP_PORT = 4647;

// xTasks stack sizes (in words)
static const uint32_t TASK_STACK_DEPTH = 3072;
static const uint32_t UDP_STACK_DEPTH = 3072;
static const uint32_t TCP_STACK_DEPTH = 3072;
static const uint32_t WSS_STACK_DEPTH = 3072;

// xQueue size (in elements)
static const uint32_t QUEUE_SIZE = 128;

#endif // _PROJECT_SETTINGS_H
