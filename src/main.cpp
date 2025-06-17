#include <Arduino.h>
#include <FrequencyReader.h>

#define DEBUG_FREQ 1 // SET TO 0 OUT TO REMOVE TRACES

#if DEBUG_FREQ
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__);
#define D_print(...) Serial.print(__VA_ARGS__)
#define D_write(...) Serial.write(__VA_ARGS__)
#define D_println(...) Serial.println(__VA_ARGS__)
#define D_SerialEnd(...) Serial.end(__VA_ARGS__);
#else
#define D_SerialBegin(...)
#define D_print(...)
#define D_write(...)
#define D_println(...)
#define D_SerialEnd(...)
#endif

void setup()
{
    Serial.begin(115200); 
    delay(1500); // Wait for serial to start up
    setupFastADC();
    setupTimerTC4();
    setupDAC();
    pinMode(2, OUTPUT);
}

void loop()
{
    runZeroCrossing();
    // Every second print out sample rate of ADC (How often it was called)
    runPrintDebug();
    if (freq > 50.025 || freq < 49.975)
    {
        freq > 50.025 ? digitalWrite(2, HIGH) : digitalWrite(2, LOW);
    }
    runComputeRMS();
    
}