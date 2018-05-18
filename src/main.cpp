//
// Created by Gustaw on 18-May-18.
//
#include <avr/io.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SoftwareSerial.cpp>


#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
extern "C"
{
extern void output_grb_strip(uint8_t *ptr, uint16_t count);
}
uint8_t arr[24]={0};
int k=0;


void setup()
{
    DDRA |= 1 << PA0;
   // output_grb_strip(arr, sizeof(arr));
    Serial.begin(115200);
}

void loop()
{
    if(Serial.available()>1)
    {
        uint8_t a = Serial.read();
        if (a==0xFA)
        {
            uint8_t b = Serial.read();
            for(int i=0;i<8;i++) {
                if (CHECK_BIT(b, i))
                {
                    arr[k] = {0xFF};
                    k++;
                    arr[k]={0xFF};
                    k++;
                    arr[k] ={0xFF};
                    k++;
                }
                else {
                    arr[k] = {0x00};
                    k++;
                    arr[k]={0x00};
                    k++;
                    arr[k] ={0x00};
                    k++;
                }

            }
            k=0;
            output_grb_strip(arr, sizeof(arr));
        }
    }
}