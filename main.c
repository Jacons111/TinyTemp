// main.c

#include <avr/wdt.h> // sleep
#include <avr/io.h>
#include "tinyOLED.h"
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

// Created by Jacob Constable, student of Umass Amherst 4/1/24
// Majority of functions provided by Professor D. McLaughlin
// This code is the attiny85 version of the MegaTemp (atmega328p) 
// Project Temperature Sensor- Tmp36, Oled Display, Attiny85, AVR Pocket Programmer, Sparkfun Serial to USB, 1700 - 3302E regulator 

//           Left     ATmega328p pins     Right
// PB5 (Reset) - AVR (Reset)   |  VCC                - 
// PB3 (RXD)   - None          |  PB2 (SCK) & (SCL)  - AVR (SCK) & Oled (SCL)
// PB4 (TXD)   - Tmp36 (Input) |  PB1 (MISO)         - AVR (MISO)
// GND         -               |  PB0 (MOSI) & (SDA) - AVR (MOSI) & Oled (SDA)

#define NUMSAMPLES 25  // #ADC Samples to average 
#define VREF 1.1        // ADC reference voltage
#define MAXTEMP 60      // Too-Hot LED turns on at this temp (Deg F)

void adc_init(void);
unsigned int get_adc(void);
void WDT_OFF(void); // shut off attiny85 and saves power 

// messages to print on OLED
const char Message1[] PROGMEM = "TEMP";
const char Message2[] PROGMEM = "F";
const char Message3[] PROGMEM = "C";
const char Message4[] PROGMEM = "TOO HOT";
const char Message5[] PROGMEM = "       ";

int main(void){
  WDT_OFF(); // sleep Make sure the WDT is off at startup
  char buffer[3];

  float tempC, tempF;
  unsigned char too_hot; // add switch_state,
  unsigned int digitalValue, tempf10, tempc10;
  unsigned long int totalValue;

  adc_init(); // add
  OLED_init(); // initialize the OLED
  OLED_clear();

  while (1){ // loop forever 
    
    totalValue = 0;
    int i;
    for (i = 0; i < NUMSAMPLES; i++)
      totalValue += get_adc(); // Get a sample from temp sensor
    digitalValue = totalValue / NUMSAMPLES;
        
    // Convert to temp & analog voltage in mV
    tempC = digitalValue * 1.0 * VREF / 10.24 - 50.; 
    tempc10 = tempC * 10.;
    tempF = tempC * 9. / 5. + 32.;
    tempf10 = tempF * 10.;

    too_hot = (tempF > MAXTEMP);
    
    OLED_cursor(20, 0);    // set cursor position
    OLED_printP(Message1); // print message 1

    unsigned char a;
    for (a = 0; a < strlen(buffer); a++){
      OLED_printC(buffer[a]);
    }
// F
    OLED_cursor(20, 2);
    OLED_printC(tempf10 / 100 + '0');     // hundreds Digit
    OLED_printC(tempf10 / 10 % 10 + '0'); // tens digit
    OLED_printC('.');
    OLED_printC(tempf10 % 10 + '0'); // 10's digit
    OLED_printP(Message2);
// C
    OLED_cursor(20, 3);
    OLED_printC(tempc10 / 100 + '0');     // hundreds Digit
    OLED_printC(tempc10 / 10 % 10 + '0'); // tens digit
    OLED_printC('.');
    OLED_printC(tempc10 % 10 + '0'); // 10's digit
    OLED_printP(Message3);

    if (too_hot){
      //PORTB |= (1 << TOO_HOT_LED); // MAXTEMP LED ON
      OLED_cursor(20, 1);
      OLED_printP(Message4);
    } else {
      //PORTB &= ~(1 << TOO_HOT_LED); // MAXTEMP LED OFF
      OLED_cursor(20, 1);
      OLED_printP(Message5);
    }

    wdt_enable(WDTO_8S);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
   
    while (1){
       sleep_mode();
    }

    return 0;

  }
}

void adc_init(void) // different from atmega328p
{
    ADMUX = 0b10000010; // Select ADC2; Vref=1.1
    ADCSRA = 0b10000011; // Enable ADC; divide by 8 for 1 MHZ clock
}

// Read ADC value
unsigned int get_adc()
{
    ADCSRA |= (1 << ADSC); // Start ADC conversion
    while ((ADCSRA & (1 << ADIF)) == 0);                       // Wait till ADC finishes
    return ADCL | (ADCH << 8);  // Read ADCL first !
}

// Watch Dog Timer shuts off attiny85 and saves power
void WDT_OFF()
{
	MCUSR &= ~(1 << WDRF);
	WDTCR |= (1 << WDCE) | (1 << WDE);
	WDTCR = 0x00;
}


