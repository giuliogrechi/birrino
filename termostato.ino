
//commentare e decommentare se necessario
// #define DEBUG
#define TEMP_FIXED 17.0
// #define lm34 A0
#define DS18B20 8
#define STANDALONE


#ifndef TEMP_FIXED
#define potentiometer A1
#endif

#ifdef STANDALONE
#define VOLTAGE_REFERENCE 1.069 //misurato con multimetro
#define pinFrigo 2 //pin 2 su standalone
#define ledFrigo 13
#else
#define VOLTAGE_REFERENCE 1.083
#define pinFrigo 13
#endif

#define MIN_TEMP 0
#define MAX_TEMP 30
#define MAX_RUN_TIME 600000 //10*60*1000 millisecondi (10 minuti)
#define SLEEP_TIME 120000    //1*60*1000 (2 minuti)

#ifdef DS18B20
#include <OneWire.h>
#include "DallasTemperature.h"
OneWire  ds(DS18B20);  // on pin DS18B20 (a 4.7K resistor is necessary)
DallasTemperature sensors(&ds);
#define SENSOR_TYPE "DS18B20"
#else
#define SENSOR_TYPE "LM34"
#endif

void setup()
{
    #ifdef DEBUG
        {Serial.begin(57600);}
    #endif
    pinMode(pinFrigo, OUTPUT);
    digitalWrite(pinFrigo, LOW);
    #ifdef STANDALONE
       {pinMode(ledFrigo, OUTPUT);
        digitalWrite(ledFrigo, LOW);}
    #endif
    #ifndef lm34
    analogReference(EXTERNAL);
    #endif
}

float setTemperature(void)
#ifdef TEMP_FIXED
{
    return TEMP_FIXED;
}
#else
{
    return mapf(analogRead(potentiometer), MIN_TEMP, MAX_TEMP);
}
#endif


#ifndef TEMP_FIXED
float mapf(int x, int out_min, int out_max)
{
  return (float)((x) * (out_max - out_min) / (1023.0) + out_min);
}
#endif

float Temperature()
#ifdef DS18B20
{
    sensors.setWaitForConversion(false);  // makes it async
    sensors.requestTemperatures();
    sensors.setWaitForConversion(true);

    int resolution = 9;
    delay(750/ (1 << (12-resolution)));
    return sensors.getTempCByIndex(0);
}
#else
{
    analogReference(INTERNAL);
    for(int i=0;i < 3;i++)
        {analogRead(lm34);} //per stabilizzare l'adc
    delay(5);
    float analogValue = 0.0;
    for(int i=0;i < 16;i++)
    {
        analogValue += analogRead(lm34);
        delay(5);
    }
    analogValue = analogValue/16.0;
    float celsius = ((((analogValue/1024.0)*VOLTAGE_REFERENCE)/0.010) - 32.0)/1.8; // 10mV per grado Fahrenheit
    // float celsius = (fahrenheit - 32.0)/1.8; //basta la precisione al grado, decimali troncati
    analogReference(DEFAULT);
    for(int i=0;i < 3;i++)
        {analogRead(lm34);} //per stabilizzare nuovamente l'adc
    return celsius;
}
#endif

unsigned long int lastStop = millis() - SLEEP_TIME;
unsigned long int lastStart = millis();

void setFrigo(boolean statusFrigo)
{

    if(statusFrigo == true)
    {
        if(digitalRead(pinFrigo) == 0 && (millis() - lastStop) > SLEEP_TIME)
        {
            #ifdef DEBUG
                {Serial.println("acceso");}
            #endif
            digitalWrite(pinFrigo, HIGH);
            lastStart = millis();
        }

        if((digitalRead(pinFrigo) == 1) && (millis() - lastStart) > MAX_RUN_TIME)
        {
            digitalWrite(pinFrigo, LOW);
            lastStop = millis();
            #ifdef DEBUG
                {Serial.println("riposoFrigo");}
            #endif
        }
    }
    else if (statusFrigo == false)
    {
        if(digitalRead(pinFrigo) == 1)
        {
            digitalWrite(pinFrigo, LOW);
            #ifdef DEBUG
                {Serial.println("spento");}
            #endif
        }
    }
}

boolean runningFlag = false;
void loop()
{
    float tempSet = setTemperature();
    float tempRead = Temperature();
    if((tempRead - tempSet) > 1.00 && runningFlag == false)
    {
        runningFlag = true;
        #ifdef STANDALONE
            {digitalWrite(ledFrigo, HIGH);}
        #endif
    }
    else if ((tempSet - tempRead) > 1.00 && runningFlag == true)
    {
        runningFlag = false;
        #ifdef STANDALONE
            {digitalWrite(ledFrigo, LOW);}
        #endif
    }

    setFrigo(runningFlag);

    #ifdef DEBUG
        {Serial.print("Temp. impostata: ");
        Serial.println(tempSet);
        Serial.print(SENSOR_TYPE);
        Serial.print(" temp. letta: ");
        Serial.println(tempRead);
        Serial.println();
        delay(1000);}
    #endif

}
