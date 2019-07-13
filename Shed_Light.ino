/*
 * Tom's Shed light project:
 *  - When the solar panel voltage >10 volts this circuit runs a boost converter to pump charge into the LiPo battery
 *  - If the LiPo is charged a MOSFET is enabled allowing the battery to be used for the lights in the shed, otherwise we disable this MOSFET
 *  - If the LiPo is fully charged we stop the boost converter to prevent overcharging
 *  - If solar is insufficient, we sleep the microcontroller to draw as little power as possible
 *  - 
 * 
 */
 
// **** INCLUDES *****
#include "LowPower.h"
#include "Average.h"

// **** DEFINES ******
#define SOLAR_THRESHOLD_VOLTAGE_LOW       12.0
#define SOLAR_THRESHOLD_VOLTAGE_HIGH      14.0
#define BATTERY_FULL_VOLTAGE_LOW          23.0
#define BATTERY_FULL_VOLTAGE_HIGH         24.0
#define BATTERY_EMPTY_VOLTAGE_LOW         20.0
#define BATTERY_EMPTY_VOLTAGE_HIGH        21.0

#define MPPT_VOLTAGE_LOW                  16.75
#define MPPT_VOLTAGE_HIGH                 17.25

#define LIGHTS_ENABLE_PIN                 8
#define BOOST_CONVERTER_PWM_PIN           9
#define INDICATOR_LED_PIN                 10
#define SOLAR_VOLTAGE_ADC_PIN             A2
#define BATTERY_VOLTAGE_ADC_PIN           A3

#define MAX_PWM_DUTY                      40
#define VOLTAGE_CONSTANT                  0.007833278    //(1.0 / 1024) * (330 + 47) / 47

//#define DEBUG

// **** Function declarations *****
void Print_Values_to_USART();
void Power_Down_No_Solar();
void Power_Down_Full_Battery();
void Read_Voltages();
float read_vcc();

// **** Variables *****
float Solar_Voltage = 0.0;
float Battery_Voltage = 0.0;
float Vcc;

Average FilterSolar(0);
Average FilterBattery(0);

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void setup()
{
  // Configure filter constants
  FilterSolar.Set_Fraction_Filter_Const(0.8, 0.2);
  FilterBattery.Set_Fraction_Filter_Const(0.98, 0.02);
  FilterBattery.Set_Fraction_Filter(20.0);
  FilterSolar.Set_Fraction_Filter(13.0);
  
  // Configure timer1 to output PWM 125KHz
  ICR1 = 128;
  TCCR1A = 0b10000010;                                                       
  TCCR1B = 0b00011001;
  OCR1A = 0;

  // Configure our pins
  pinMode(BOOST_CONVERTER_PWM_PIN, OUTPUT);  
  pinMode(INDICATOR_LED_PIN, OUTPUT);
  pinMode(LIGHTS_ENABLE_PIN, OUTPUT);

  #ifdef DEBUG
    // Start USART
    Serial.begin(115200);
    Serial.println("Starting Up...");
  #endif
}

void loop() 
{
  LowPower.idle(SLEEP_60MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
  
  // Flash our LED to indicate we're turned on
  static byte LED_counter = 0;
  if(LED_counter++ % 6 == 0)
  {
    Print_Values_to_USART();
    digitalWrite(INDICATOR_LED_PIN, HIGH);
    delay(5);
    digitalWrite(INDICATOR_LED_PIN, LOW);
  }

  Read_Voltages();

  // Here we go into a power saving mode if the battery is fully charged or there is no solar power
  if(Battery_Voltage > BATTERY_FULL_VOLTAGE_HIGH)
  {
    Power_Down_Full_Battery();
  }
  else if(Solar_Voltage < SOLAR_THRESHOLD_VOLTAGE_LOW)
  {
    Power_Down_No_Solar();
  }

  // Ramp up or down the boost converter PWM duty to adjust the current we draw from the solar panel to optimise power harvested
  if(Solar_Voltage > MPPT_VOLTAGE_HIGH and OCR1A < MAX_PWM_DUTY)
  {
    OCR1A = OCR1A + 1;
  }
  else if(Solar_Voltage < MPPT_VOLTAGE_LOW and OCR1A > 0)
  {
    OCR1A = OCR1A - 1;
  }    
}

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void Read_Voltages()
{
  Vcc = read_vcc();    // We have to read the bandgap voltage several times in order for the 14pF ADC capacitor to charge up
  Vcc = read_vcc(); 
  Vcc = read_vcc();
  Vcc = read_vcc();
  
  Battery_Voltage = (float)analogRead(BATTERY_VOLTAGE_ADC_PIN) * VOLTAGE_CONSTANT * Vcc;
  Battery_Voltage = FilterBattery.Fraction_Filter(Battery_Voltage);
  
  Solar_Voltage = (float)analogRead(SOLAR_VOLTAGE_ADC_PIN) * VOLTAGE_CONSTANT * Vcc;

  if(OCR1A > 4)       // If the boost converter is active filter the solar pannel voltage readings somewhat
  {
    Solar_Voltage = FilterSolar.Fraction_Filter(Solar_Voltage);
  }
  else
  {
    FilterSolar.Set_Fraction_Filter(Solar_Voltage);
  }

  // Here we allow the lights to be turned on if the battery is charged enough. With hysterisis
  if(Battery_Voltage > BATTERY_EMPTY_VOLTAGE_HIGH)
  {
    digitalWrite(LIGHTS_ENABLE_PIN, LOW);    
  }
  else if(Battery_Voltage < BATTERY_EMPTY_VOLTAGE_LOW)
  {
    digitalWrite(LIGHTS_ENABLE_PIN, HIGH);
  }  
}

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void Print_Values_to_USART()
{
  #ifdef DEBUG
    Serial.print(Battery_Voltage);
    Serial.print(", ");
    Serial.print(Solar_Voltage);
    Serial.print(", ");
    Serial.print(Vcc);
    Serial.print(", ");
    Serial.print(OCR1A);
    Serial.println(" ");
    Serial.flush();
  #endif
}

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
float read_vcc()
{
    const float V_BAND_GAP = 1.1;           // typical
    ADMUX  = _BV(REFS0)                     // ref = Vcc
           | 14;                            // channel 14 is the bandgap reference
    ADCSRA |= _BV(ADSC);                    // start conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);  // wait until complete
    return V_BAND_GAP * 1024 / ADC;
}

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void Power_Down_No_Solar()
{ 
  OCR1A = 0;
  pinMode(BOOST_CONVERTER_PWM_PIN, INPUT);                                    // Disables the Boost converter
  
  while(Solar_Voltage < SOLAR_THRESHOLD_VOLTAGE_HIGH)
  {
    // Enter power down state for 8 s with ADC and BOD module disabled
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);  
    
    // Flash LED
    digitalWrite(INDICATOR_LED_PIN, HIGH);
    delay(5);
    digitalWrite(INDICATOR_LED_PIN, LOW);

    // Perform an ADC Read of Solar cell voltage and put it through our averaging filter.
    Read_Voltages();
    Print_Values_to_USART();
  }

  pinMode(BOOST_CONVERTER_PWM_PIN, OUTPUT);                                 // Enables the boost converter 
}

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void Power_Down_Full_Battery()
{ 
  OCR1A = 0;
  pinMode(BOOST_CONVERTER_PWM_PIN, INPUT);                                    // Disables the Boost converter
  
  while(Battery_Voltage > BATTERY_FULL_VOLTAGE_LOW)
  {
    // Enter power down state for 2 s with ADC module disabled
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_ON);     

    // Flash LED
    digitalWrite(INDICATOR_LED_PIN, HIGH);
    delay(2000);
    digitalWrite(INDICATOR_LED_PIN, LOW);

    // Perform an ADC Read of Solar cell voltage and put it through our averaging filter.
    Read_Voltages();
    Print_Values_to_USART();
  }
  
  pinMode(BOOST_CONVERTER_PWM_PIN, OUTPUT);                                 // Enables the boost converter 
}











