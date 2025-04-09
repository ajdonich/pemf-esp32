#include "SSD1306Wire.h" 


// Serial
#define BAUD_RATE 9600

// IO Pins 
#define DAC_PIN 25  // GPIO25 => DAC 1
#define ADC_PIN 13  // GPIO13 => ADC2_4

// Initialize the OLED display using Wire library
SSD1306Wire display(0x3c, SDA, SCL);

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) delay(100);

    display.init();
    display.setContrast(255);
    display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    display.flipScreenVertically();
    display.clear();
}

void writeDisplay(const String &btext, const String &ytext) {
    static int centerX = 128 / 2;
    static int blueCenterY = ((64 - 16) / 2) + 16; 
    static int yellowCenterY = 16 / 2; 

    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(centerX, blueCenterY, btext);

    display.setFont(ArialMT_Plain_10);
    display.drawString(centerX, yellowCenterY, ytext);
    display.display();
}

void safeDacWrite(uint8_t pin, uint8_t value) {
    if (0 <= value && value <= 255) dacWrite(pin, value);
    else Serial.printf("ERROR: bad dacWrite voltage: %u\n", value);
}

void loop() {
    static double omega = 2.0 * PI * 0.05; // rad/sec

    // static int voltageIndex = 0;
    static float adc_to_voltage = 3.3 / 4095.0;
    static float dac_to_voltage = 3.3 / 255.0;
    static struct timeval tv_now;

    gettimeofday(&tv_now, NULL);
    double t = (double)((int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec) * 1e-6;
    uint8_t vindex = (uint8_t)(255.0 * (0.5*(cos(omega * t) + 1)));
    float vout = (float)vindex * dac_to_voltage;
    delay(100);

    // Serial.printf("vindex: %u, vout: %f\n", vindex, vout);

    // // Write and display output signal
    // String text = "Vout: " + String(vout) + " V\n";
    safeDacWrite(DAC_PIN, vindex);
    // delay(100);

    // // Read and display input signal
    // float vin = (float)analogRead(ADC_PIN) * adc_to_voltage;
    // text += "Vin: " + String(vin) + " V";
    // String err = "Error: " + String(vin-vout);
    // writeDisplay(text, err);
}