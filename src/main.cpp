#include <Arduino.h>
#include <board.h>
#include <TimerInterrupt_Generic.h>

#define DAC_pin PIN_PA03
#define ADC_pin PIN_PB02
#define tempProbe PIN_PB03

// TC3, TC4, TC5 max permissible TIMER_INTERVAL_MS is 1398.101 ms, larger will overflow, therefore not permitted
// Use TCC, TCC1, TCC2 for longer TIMER_INTERVAL_MS
#define TIMER_INTERVAL_MS        1000

#define TIMER_DURATION_MS        5000

volatile uint32_t preMillisTimer = 0;

SAMDTimer ITimer(TIMER_TC3);

void TimerHandler()
{
    static bool toggle = false;
    digitalWrite(LED_BUILTIN, toggle);
    toggle = !toggle;
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);

    while (!Serial && millis() < 5000);

    delay(100);

    Serial.print(F("\nStarting TimerInterruptTest on "));
	Serial.println(BOARD_NAME);
	Serial.println(SAMD_TIMER_INTERRUPT_VERSION);
	Serial.println(TIMER_INTERRUPT_GENERIC_VERSION);
	Serial.print(F("CPU Frequency = "));
	Serial.print(F_CPU / 1000000);
	Serial.println(F(" MHz"));

    // Interval in millisecs
	if (ITimer.attachInterruptInterval_MS(TIMER_INTERVAL_MS, TimerHandler))
	{
		preMillisTimer = millis();
		Serial.print(F("Starting ITimer OK, millis() = "));
		Serial.println(preMillisTimer);
	}
	else
		Serial.println(F("Can't set ITimer. Select another freq. or timer"));
}

void loop()
{
    // Do nothing
}

