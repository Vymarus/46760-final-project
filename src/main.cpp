#include <Arduino.h>
#include <LiquidCrystal.h>
#include <FrequencyReader.h>
#include "arduino_secrets.h"
#include "thingProperties.h"

void updateLCD();

// LCD initlization: (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
float frequency = 50.000;

const int mode_button_Pin = A4; // Pin for mode button
bool last_mode_state = HIGH;	// phyisical state
bool button_mode_state = false; // mode state

const int led_button_Pin = A5; // Pin for LED button
bool last_led_state = HIGH;	   // last physical state
bool button_led_state = false; // mode state

const int LEDPin_ext = A3; // Pin for external LED
boolean changes = false;

void setup()
{
	Serial.begin(9600);
	for(unsigned long const serialBeginTime = millis(); !Serial && (millis() - serialBeginTime <= 5000); ) { }

	// intialize lcd
	lcd.begin(16, 2);

	pinMode(mode_button_Pin, INPUT_PULLUP); // Mode button with internal Pull-Up
	pinMode(led_button_Pin, INPUT_PULLUP);	// LED button with internal Pull-Up
	pinMode(LED_BUILTIN, OUTPUT);			// LED internal
	pinMode(LEDPin_ext, OUTPUT);			// LED external

	initProperties();
	ArduinoCloud.begin(ArduinoIoTPreferredConnection, false);
	setDebugMessageLevel(2);
	ArduinoCloud.printDebugInfo();

	// display setup values which stay consitent throughout program
	lcd.setCursor(0, 0);
	lcd.print("Frequency:50.000 ");
	lcd.setCursor(0, 1);
	lcd.print("LED:off Mode:off");

	// frequency setup
	setupFastADC();
	setupTimerTC4();
	setupDAC();
	pinMode(0, OUTPUT);
}

void loop()
{
	static uint32_t last_millis = millis();
	if (millis() > last_millis+1000) 
	{
		ArduinoCloud.update(); // update cloud
		Serial.println("Cloud Update");
	}

	// physical state of each button
	bool currentPhysicalState_mode = digitalRead(mode_button_Pin);
	bool currentPhysicalState_led = digitalRead(led_button_Pin);

	// only changes if physical state changes
	if (currentPhysicalState_mode != last_mode_state)
	{
		last_mode_state = currentPhysicalState_mode; //

		// from High to Low
		if (currentPhysicalState_mode == LOW)
		{
			button_mode_state = !button_mode_state; // change state of exclusive internal var, is needed to sync dashboard with                                                      pyhsical button
			mode = button_mode_state;				// assign mode with button variable
			Serial.print("Mode button toggled: ");	// serial output
			Serial.println(mode);

			changes = true;
		}
	}

	// only if 2nd physical state changes
	if (currentPhysicalState_led != last_led_state)
	{
		last_led_state = currentPhysicalState_led;
		// only from High to Low
		if (currentPhysicalState_led == LOW)
		{
			button_led_state = !button_led_state; // change state of internal variable to stay sync with dashboard
			button_led = !button_led_state;		  // inverse only for dashboard
			Serial.print("LED button toggled: ");
			Serial.println(button_led);

			led_external = !led_external;				 // switch light on or off
			Serial.print("LED Button toggled LED to: "); // serial output
			Serial.println(led_external);

			// Update LED-Zustand auf dem LCD
			changes = true;
		}
	}

	// control internal LED
	digitalWrite(LED_BUILTIN, led_internal); // write LEDs

	// control external LED
	digitalWrite(LEDPin_ext, led_external);

	// delay(200); //debounce button
	if (changes == true)
	{
		updateLCD();
		changes = false;
	}

	// frequency loop code
	runZeroCrossing();
	// Every second print out sample rate of ADC (How often it was called)
	runPrintDebug();
	// if (freq > 50.025 || freq < 49.975)
	// {
	// 	freq > 50.025 ? digitalWrite(LED_BUILTIN, HIGH) : digitalWrite(LED_BUILTIN, LOW);
	// }
	runComputeRMS();
}

// functions to change value through dashboard and log to serial
void onLedExternalChange()
{
	// Add your code here to act upon LedExternal change
	Serial.print("Led status changed: ");
	Serial.println(led_external);
	changes = true;
}

void onModeChange()
{
	// Add your code here to act upon Mode change
	Serial.print("Mode changed via dashboard: ");
	Serial.println(mode);
	digitalWrite(mode_button_Pin, mode);
	changes = true;
}

void updateLCD()
{
	lcd.setCursor(0, 0);
	lcd.print("                "); // Leere erste Zeile
	lcd.setCursor(0, 1);
	lcd.print("                "); // Leere zweite Zeile

	lcd.setCursor(0, 0);
	lcd.print("Frequency:");
	lcd.print(freq, 3);
	lcd.setCursor(0, 1);
	lcd.print("LED:");
	lcd.print(led_external ? "on " : "off");
	lcd.print(" Mode:");
	lcd.print(mode ? "on " : "off");
}

void onLedInternalChange()
{
	// Add your code here to act upon LedInternal change
}