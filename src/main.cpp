//
// Created by Gustaw on 18-May-18.
//
#include <avr/io.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SoftwareSerial.cpp>


#define FLAG_SERIAL_RX 0xAA
#define FLAG_SERIAL_TX 0xAB
#define FLAG_SERIAL_MASTER_SETUP 0xBA
#define FLAG_SERIAL_SLAVE_SETUP 0xBB
#define FLAG_SLAVE_RECEIVE 0xCA
#define FLAG_PC_READY_TO_RECEIVE 0xCB
#define FLAG_PC_NORMAL_TRANSMIT



uint8_t FLAG_READY_TO_TRANSMIT = 0;
uint8_t FLAG_PC_DATA_RECEIVED = 0;
uint8_t FLAG_READY_TO_RECEIVE_FROM_SLAVE = 0;
uint8_t FLAG_SLAVE_DATA_RECEIVED = 0;

extern "C"
{
    #define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
    extern void output_grb_strip(uint8_t *ptr, uint16_t count);
}
uint8_t arr[24]={0};
int k=0;

void handleSerialRx()
{
    unsigned char first_flag = Serial.read();
    switch(first_flag)
    {
        case FLAG_SERIAL_MASTER_SETUP:
            break;
        case FLAG_SERIAL_SLAVE_SETUP:
            break;
        case FLAG_SLAVE_RECEIVE:
            break;
        case FLAG_PC_READY_TO_RECEIVE:
            FLAG_READY_TO_TRANSMIT = 1;
            break;
        case FLAG_PC_NORMAL_TRANSMIT:

            FLAG_PC_DATA_RECEIVED = 1;
            break;




        default:
            break;
    }


}









void setup()
{
    //DDRA |= 1 << PA0;
   // output_grb_strip(arr, sizeof(arr));
    Serial.begin(115200);
}







void loop()
{
    if(Serial.available()>=2)
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


    if(FLAG_READY_TO_TRANSMIT)//transmit data to PC
    {

    }
    if(FLAG_PC_DATA_RECEIVED)
    {
        //send to slaves
        //finish sending
    }
    if(FLAG_READY_TO_RECEIVE_FROM_SLAVE)
    {
        //rx from slave
        FLAG_READY_TO_RECEIVE_FROM_SLAVE = 0;
        FLAG_SLAVE_DATA_RECEIVED = 1;
    }
}