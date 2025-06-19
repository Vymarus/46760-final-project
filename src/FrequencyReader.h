#ifndef __FREQUENCYREADER_H__
#define __FREQUENCYREADER_H__

#include <Arduino.h>

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

#define DAC_VREF        3.3f  // this is the value of AVCC, the reference voltage
#define DAC_MASK        0x3FF // DAC is 10 bit, so this is the maximum value
#define DAC_FACTOR      (DAC_MASK / DAC_VREF)  // For converting to voltage

volatile uint32_t current_time = 0;
volatile uint32_t ADC_reads = 0;
volatile uint32_t ADC_data = 0;
volatile uint32_t ADC_data_last = 0;
volatile uint32_t ADC_period = 0;
volatile uint32_t ADC_period_last = 0;
volatile uint8_t zero_crossing_detect = 0;
volatile uint8_t fresh_data = 0;
volatile uint64_t period;
volatile uint64_t last_period;
uint32_t RMS_value = 0;
float RMS_voltage = 0;
float freq;
unsigned int zeroCross = 511;


void DAC_setRaw(const uint16_t value) {
    // Disable output
    // DAC->CTRLB.bit.EOEN = 0;
    // DAC->CTRLB.bit.IOEN = 0;
    // DAC->CTRLA.reg = DAC_CTRLA_ENABLE;
    // while (1 == DAC->STATUS.bit.SYNCBUSY);
    DAC->DATABUF.reg = DAC_MASK & value;
    while (1 == DAC->STATUS.bit.SYNCBUSY);
    // Enable output
    // DAC->CTRLB.bit.IOEN = 1;
}

// This function acts as an lowpass filter to filter data.
#define f_c 50
#define RC (1/(2 * 3.14 * f_c))
uint32_t LowpassFilter(uint32_t sample, uint16_t period)
{
    // Initial filter value
    static uint32_t prev_sample = 512;
    const uint32_t alpha = period / (RC + period);
    uint32_t sample_filter = (alpha * sample + (1-alpha)*prev_sample);
    prev_sample = sample;
    return (sample_filter);
}

#define INITIAL 920 // Initial filter value
#define SAMPLES 8192
// Run a RMS filter so RMS is calculated runningly
uint32_t RMS_filter(uint16_t sample)
{
    // Initial filter setup
    static uint16_t rms = INITIAL;
    static uint64_t sum_squares = 1UL * SAMPLES * INITIAL * INITIAL;
    // 
    sum_squares -= sum_squares / SAMPLES;
    sum_squares += (uint64_t) sample * sample;
    if (rms == 0) rms = 1; // Dont divide by zero
    rms = (rms + sum_squares / SAMPLES / rms) / 2;
    return rms;

    // Divide by the period
}

// ADC handler that runs every time data is ready in the ADC
void ADC_Handler()
{
    ADC_data = ADC->RESULT.reg;
    ADC_period = TC4->COUNT32.COUNT.reg;
    ADC_data = LowpassFilter(ADC_data, (ADC_period-ADC_period_last));
    // D_println(ADC_data);
    if ((ADC_data >= zeroCross) && (ADC_data_last < zeroCross)) // Zero crossing detected
    {
        period = ADC_period;
        zero_crossing_detect = true;
    }
    ADC_reads++;

    // Consider if this take to long
    DAC_setRaw(ADC_data);

    ADC_data_last = ADC_data;
    ADC_period_last = ADC_period;
    fresh_data = 1;
}

void setupFastADC()
{
    // Enable the clock fo ADC
    PM->APBCMASK.reg |= PM_APBCMASK_ADC;

    ADC->CTRLA.bit.ENABLE = 0; // Disable ADC
    while (ADC->STATUS.bit.SYNCBUSY == 1);          // Wait for synchronization

    // Setup generic clocks source clock
    D_println("Setting generic clock");
        GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN |        // Enable GCLK4
                        GCLK_GENCTRL_SRC_DFLL48M |  // Set to external 48MHz
                        // GCLK_GENCTRL_IDC |          // In case of odd division, improve it by using 50/50.
                        GCLK_GENCTRL_ID(4);         // Select GCLK5
    while (GCLK->STATUS.bit.SYNCBUSY); // Wait for synchronization

    // Setting up generic clock division
    D_println("Setting clock");
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(10) |    // Divide the main clock down. Max 255 for clock 3 - 8
                       GCLK_GENDIV_ID(4);       // Select Generic Clock (GCLK) 4
    while (GCLK->STATUS.bit.SYNCBUSY); // Wait for synchronization

    // Feed Generic clock to ADC
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable clock
                        GCLK_CLKCTRL_GEN_GCLK4 |    // Select Generic Clock (GCLK) 4
                        GCLK_CLKCTRL_ID_ADC;        // Feed it to the ADC
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Setup prescaler of ADC. Initial Clock is 48MHz / 15 = 3.2MHz
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 |    // Divide Clock by 8.
                     ADC_CTRLB_RESSEL_10BIT |       // Result on 10 bits
                     ADC_CTRLB_FREERUN;             // Continous ADC
    while (ADC->STATUS.bit.SYNCBUSY);  

    // Setup reference to use 1/2 VDDNA, which should be 1.65V
    ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1; 
    while (ADC->STATUS.bit.SYNCBUSY);

    // Setup sample amount. Refer to 32.6.7 for selection of values
    // Averaging adds samples together. Adjust then left shifts (divide) the result
    // Higher samplenumber will automaticaly shift, so ADJRES max value is 0x04.
    ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_16 |    // 8 sample
                       ADC_AVGCTRL_ADJRES(0x04ul);  // Adjusting result by 3
    while (ADC->STATUS.bit.SYNCBUSY);

    // Controls how long sampling is done for
    // Sampling time = (SAMPLEN + 1) * (CLK_ADC/2)
    ADC->SAMPCTRL.reg = 0x03;                       // Sampling Time Length = 4
    while (ADC->STATUS.bit.SYNCBUSY);

    // Setup Input parameters. A1 is PB02, and PB02 ADC input is AIN[10]. Therefore pin 10
    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 |      // result looks to be double, so half
                         ADC_INPUTCTRL_MUXPOS_PIN10 |
                         ADC_INPUTCTRL_MUXNEG_GND; 
    while (ADC->STATUS.bit.SYNCBUSY);

    // Setup pins to sample Pin A1, PB02 in variant.h
    PORT->Group[PORTB].DIRCLR.reg = PIN_A1;

    PORT->Group[PORTB].PINCFG[g_APinDescription[PIN_A1].ulPin].reg |= PORT_PINCFG_PMUXEN;

    // Function B is ADC. Since PB02 is even, PMUX: 02 / 2 = 1 is used
    // For odd this would be (pin - 1) / 2.
    PORT->Group[PORTB].PMUX[1].reg = PORT_PMUX_PMUXE_B;


    // Setup pin using pinPeripheral
    // pinPeripheral(A1, PIO_ANALOG);

    // Correction values based on the CorrectADCResponse sketch
    ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(1); // Offset 1

    ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(2051);    // Gain 2051

    // Enable digital correction logic
    ADC->CTRLB.bit.CORREN = 1;
    while(ADC->STATUS.bit.SYNCBUSY);
    
    
    // Enable interrupt
    ADC->INTENSET.bit.RESRDY = 1;
    while (ADC->STATUS.bit.SYNCBUSY == 1);
    
    NVIC_EnableIRQ(ADC_IRQn);
    __enable_irq();

    // Enable ADC
    ADC->CTRLA.bit.ENABLE = 1;                      // Enable ADC
    while (ADC->STATUS.bit.SYNCBUSY == 1);          // Wait for synchronization

    D_println("Setup Complete");
}

// Setup a 10kHz timer for counting ms
void setupTimerTC4()
{
    // Enable the clock for the timer
    PM->AHBMASK.reg |= PM_APBCMASK_TC4;

    // Disable before editing
    TC5->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

    // Setup generic clock
    // Setup generic clocks source clock
    D_println("Setting generic clock");
    GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN |        // Enable GCLK5
                        GCLK_GENCTRL_SRC_OSC8M |    // Set to internal 8MHz
                        GCLK_GENCTRL_ID(5);         // Select GCLK5
    while (GCLK->STATUS.bit.SYNCBUSY); // Wait for synchronization

    // Setting up generic clock division
    D_println("Setting clock");
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(8) |   // Divide the main clock down. Max 255 for clock 3 - 8
                       GCLK_GENDIV_ID(5);       // Select Generic Clock (GCLK) 5
    while (GCLK->STATUS.bit.SYNCBUSY); // Wait for synchronization

    // Feed Generic clock to TC4 and TC5
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable clock
                        GCLK_CLKCTRL_GEN_GCLK5 |    // Select Generic Clock (GCLK) 5
                        GCLK_CLKCTRL_ID_TC4_TC5;    // Feed it to the TC4 and TC5 (paired)
    while (GCLK->STATUS.bit.SYNCBUSY);

    TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 |    // Set to count16 mode. No PER
                             TC_CTRLA_WAVEGEN_NPWM;     // Normal Wavegen (Not used)

    TC4->COUNT32.CTRLA.bit.ENABLE = 1;              // Enable TC5
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);       // Wait for synchronization

}

void setupDAC() 
{
    // Enable DAC peripheral
    PM->APBCMASK.reg |= PM_APBCMASK_DAC;

    // Disable DAC
    DAC->CTRLA.bit.ENABLE = 0;
    while (DAC->STATUS.bit.SYNCBUSY);

    // Feed GCLK5 used by TC4.
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable clock
                        GCLK_CLKCTRL_GEN_GCLK5 |    // Select Generic Clock (GCLK) 5
                        GCLK_CLKCTRL_ID_DAC;    // Feed it to the TC4 and TC5 (paired)
    while (GCLK->STATUS.bit.SYNCBUSY);

    DAC->CTRLB.reg = DAC_CTRLB_REFSEL_AVCC;    // Set to use 3.3V as refference
    while (DAC->STATUS.bit.SYNCBUSY);

    // Setup pins to output Pin A0, PA02 in variant.h
    PORT->Group[PORTB].DIRSET.reg = PIN_A0;

    PORT->Group[PORTB].PINCFG[g_APinDescription[PIN_A0].ulPin].reg |= PORT_PINCFG_PMUXEN;

    // Function B is ADC. Since PA02 pin number is even, PMUX: 02 / 2 = 1 is used
    // For odd this would be (pin - 1) / 2.
    PORT->Group[PORTB].PMUX[1].reg = PORT_PMUX_PMUXE_B; 

    // Enable the DAC
    DAC->CTRLA.bit.ENABLE = 1;
    while(DAC->STATUS.bit.SYNCBUSY);

    // Enable the output
    DAC->CTRLB.bit.EOEN = 1;
    while(DAC->STATUS.bit.SYNCBUSY);

}

void runZeroCrossing()
{
    if (zero_crossing_detect)
    {
        // Clear flag
        zero_crossing_detect = false;

        // Calculate frequency based on timer frequency being 1 MHz
        freq = 1000000.0/(float(period - last_period));

        // D_print("Period: "); D_print(period);
        // D_print(" | LastPeriod: "); D_print(last_period);
        // D_print(" | Freq: "); D_println(freq, 3);
        // D_print(" | Diference: "); D_println(period-last_period);
        
        last_period = period;
    }
}

void runPrintDebug()
{
    if (current_time < millis()-1000)
    {
        D_print("ADCReads: "); D_println(ADC_reads);
        D_print("RMS: "); D_println(RMS_voltage);
        D_print("Freq: "); D_println(freq, 3);
        ADC_reads = 0;
        current_time = millis();
    }
}

void runComputeRMS()
{
    if (fresh_data)
    {
        // Compute RMS
        RMS_value = RMS_filter(ADC_data);
        RMS_voltage = RMS_value * (3.3 / 1023.0);

        fresh_data = 0;
    }
}

#endif // __FREQUENCYREADER_H__