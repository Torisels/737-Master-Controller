//
// Created by Gustaw on 18-May-18.
//
#include <avr/io.h>
#include <Arduino.h>
extern void output_grb_strip(uint8_t * ptr, uint16_t count);

uint8_t arr[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
void setup()
{
    DDRA |= 1 << PA0;
    output_grb_strip(arr, sizeof(arr));
}

void loop()
{

}