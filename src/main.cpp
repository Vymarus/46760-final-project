#include <Arduino.h>
#include <board.h>
#include <TimerInterrupt_Generic.h>

#define DAC_pin A0
#define ADC_pin A1
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

void ADC_Handler(void)
{

}

void setup()
{
	// Setup Basic serial and light
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

	// Setup 1ms timer
    while (!Serial && millis() < 5000);

    delay(100);

    Serial.print(F("\nStarting TimerInterruptTest on "));
	Serial.println(BOARD_NAME);
	Serial.println(SAMD_TIMER_INTERRUPT_VERSION);
	Serial.println(TIMER_INTERRUPT_GENERIC_VERSION);
	Serial.print(F("CPU Frequency = "));
	Serial.print(F_CPU / 1000000);
	Serial.println(F(" MHz"));

    // Interval in millisecs, attach to TimerHandler
	if (ITimer.attachInterruptInterval_MS(TIMER_INTERVAL_MS, TimerHandler))
	{
		preMillisTimer = millis();
		Serial.print(F("Starting ITimer OK, millis() = "));
		Serial.println(preMillisTimer);
	}
	else
		Serial.println(F("Can't set ITimer. Select another freq. or timer"));

	// Setup DMAC for ADC usage
	// Idea is to use the DMAC to get ADC data into a buffer
	// The DMA should take 1 sample and put it into a 2 sized FIFO buffer
	// Every time DMA is done, the CPU should check the data, and calculate the zero crossing
	// The CPU should also check the time between zero crossings, and calculate the frequency


	// Idea is to use the DMAC to load ADC values into a FIFO buffer
	// Then the CPU check the buffer and calculates the frequency. 
	// Frequency should be checked every period of the signal, with the buffer being atleast 8 times higher
	// The ADC samplerate should be 10000sps for now. 

	// Setup ADC
	// After ADC setup up, and buffer is running, wait til buffer is full
	

}

void loop()
{
    // Do nothing
}

