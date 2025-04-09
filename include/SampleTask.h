#include "freertos/task.h"

class WssTask;

class SampleTask {
public:
    SampleTask(int adcpin_a, size_t nsamples_a);
    void run();

private:
    int adcpin;
    size_t nsamples;
    TaskHandle_t sampleTaskHandle;
    WssTask *wssTask;

    static void sampleTaskLoop( void *pvParameters );
};
