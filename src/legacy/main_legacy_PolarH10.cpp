#include <Arduino.h>
#include "BLEDevice.h"
#include "WssTask.h"

#define BAUD_RATE 9600
// #define BAUD_RATE 460800

static BLEUUID HR_SERVICE("0000180d-0000-1000-8000-00805f9b34fb");
static BLEUUID HR_CHARACTERISTIC("00002a37-0000-1000-8000-00805f9b34fb");

static BLEAdvertisedDevice* polarDevice = nullptr;
static BLEClient* polarClient = nullptr;
WssTask *wssTask = nullptr;

class : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advDevice) {
        if (advDevice.haveName()) {
            Serial.printf("%s, RSSI: %d\n", advDevice.getName().c_str(), advDevice.getRSSI());
            if (advDevice.getName() == "Polar H10 C3D72324") {
                polarDevice = new BLEAdvertisedDevice(advDevice);
                BLEDevice::getScan()->stop();
            }
        }
    }
} advertisedDeviceCb;

class : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {}

    // Free service and charateristic allocations in polarClient
    void onDisconnect(BLEClient* pClient) {
        Serial.println("BLEClientCallbacks::onDisconnect");
        delete polarDevice;
        delete polarClient; 
        polarDevice = nullptr;
        polarClient = nullptr;
    }
} clientCb;

// Return raw data buffer as hex string
std::string toHexString(uint8_t *buf, size_t len) {
    char cstr[len*2 + ((int)len/4) + 1];
    for(int i = 0, j = 0; i < len; i++) {
        if (i > 0 && (i%4) == 0) { 
            std::sprintf(&cstr[j], "%s", " ");
            j += 1;
        }
        std::sprintf(&cstr[j], "%02x", buf[i]);
        j += 2;
    }
    return std::string(cstr);
}

void onNotify(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.printf("data: %s\n", toHexString(pData, length).c_str());
    wssTask->writeMessage(QueueMessage(pData, length));
}

void scanAdvChannels() {
    BLEScan* pBLEScan = BLEDevice::getScan();
    BLEScanResults foundDevices = pBLEScan->start(5, false); // 5 sec
    Serial.printf("Devices found: %d\nScan done!\n\n", foundDevices.getCount());
    pBLEScan->clearResults();   // release memory
}

void connectClient() {
    if (polarDevice && polarDevice->isAdvertisingService(HR_SERVICE)) {
        polarClient  = BLEDevice::createClient();
        polarClient->setClientCallbacks(&clientCb);
        polarClient->connect(polarDevice);
        polarClient->setMTU(517);

        // Register for HR_CHARACTERISTIC notifications
        BLERemoteService* hrService = polarClient->getService(HR_SERVICE);
        hrService->getCharacteristic(HR_CHARACTERISTIC)->registerForNotify(onNotify);
    }    
}

// Note: assumes characteristic->canRead() == true
std::string rawToHexString(BLERemoteCharacteristic* characteristic) {
    size_t len = characteristic->readValue().length();
    uint8_t *buf = characteristic->readRawData();
    return toHexString(buf, len);
}

// Print the services and characteristics of pAdvDevice
void interrogateDevice(BLEAdvertisedDevice *pAdvDevice) {
    if (pAdvDevice->haveServiceUUID()) {
        BLEClient* pClient  = BLEDevice::createClient();
        pClient->setClientCallbacks(&clientCb);
        pClient->connect(pAdvDevice);
        pClient->setMTU(517);

        // Iterate each service in advertised device
        Serial.printf("Interrogating device: %s\n", pAdvDevice->getName().c_str());
		for (int i=0; i < pAdvDevice->getServiceUUIDCount(); i++) {
            BLEUUID uuid = pAdvDevice->getServiceUUID(i);
            BLERemoteService* pRemoteService = pClient->getService(uuid);
            if (pRemoteService == nullptr) {
                Serial.printf("Failed to get service: %s\n", uuid.toString().c_str());
                continue;
            }
            Serial.printf("  %s\n", pRemoteService->toString().c_str());

            // Iterate each charateristic in the service
            auto pMap = pRemoteService->getCharacteristics();
            std::map<std::string, BLERemoteCharacteristic*>::iterator it;
            for (it = pMap->begin(); it != pMap->end(); it++) {
                Serial.printf("    %s\n", it->second->toString().c_str());

                if(it->second->canRead()) {
                    std::string hexValue = rawToHexString(it->second);
                    Serial.printf("      Value: %s\n", hexValue.c_str());
                }
            }
        }

        pClient->disconnect();
	}
}

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) delay(100);

    wssTask = new WssTask();
    wssTask->run();

    BLEDevice::init("ESP32");
    BLEScan* pBLEScan = BLEDevice::getScan(); // static member accessor 
    pBLEScan->setAdvertisedDeviceCallbacks(&advertisedDeviceCb);
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);  // less or equal setInterval value
}

void loop() {
    if (!polarDevice) scanAdvChannels();
    else if (!polarClient) connectClient();
    delay(15 * 1000); // sleep 15 seconds
}
