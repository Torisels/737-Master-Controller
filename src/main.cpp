//
// Created by Gustaw on 18-May-18.
//
#include <avr/io.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SoftwareSerial.cpp>
#include <Wire.h>

#define FLAG_SERIAL_RX 0xAA
#define FLAG_SERIAL_TX 0xAB
#define FLAG_SERIAL_MASTER_SETUP 0xBA
#define FLAG_SERIAL_SLAVE_SETUP 0xBB
#define FLAG_SLAVE_RECEIVE 0xCA
#define FLAG_PC_READY_TO_RECEIVE 0xCB
#define FLAG_PC_NORMAL_TRANSMIT




#define SLAVE_SEND_BF_SIZE 32
#define SLAVE_RX_BF_SIZE 32

#define NUMBER_OF_SLAVES 4
#define FLAG_NEW_SLAVE 0xDD


#define BYTES_TO_RECEIVE_FROM_SLAVE 24

uint8_t FLAGS = 0x00;

#define READY_TO_TRANSMIT (1<<0)
#define PC_DATA_RECEIVED  (1<<1)
#define READY_TO_RECEIVE_FROM_SLAVE (1<<2)
#define SLAVE_DATA_RECEIVED (1<<3)

uint8_t SLAVE_SEND_DATA[SLAVE_SEND_BF_SIZE][NUMBER_OF_SLAVES] = {};
uint8_t SLAVE_RX_DATA[SLAVE_RX_BF_SIZE][NUMBER_OF_SLAVES] = {};

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
            FLAGS |= READY_TO_TRANSMIT;
            break;
        case FLAG_PC_NORMAL_TRANSMIT:

            PC_DATA_RECEIVED = 1;
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


    if(FLAGS & READY_TO_TRANSMIT)//transmit data to PC
    {

    }
    if(PC_DATA_RECEIVED)//Normal, no setup routine
    {
        for(int i=1;i<=NUMBER_OF_SLAVES;i++)
        {
            Wire.beginTransmission(i);
            Wire.write(SLAVE_SEND_DATA[i],SLAVE_SEND_BF_SIZE);
            Wire.endTransmission();
        }
        PC_DATA_RECEIVED = 0;
        READY_TO_RECEIVE_FROM_SLAVE = 1;
    }
    if(READY_TO_RECEIVE_FROM_SLAVE)
    {
        //rx from slave
        for(int i=0;i<NUMBER_OF_SLAVES;i++)
        {
            Wire.requestFrom(i,BYTES_TO_RECEIVE_FROM_SLAVE);
            uint8_t slave_rx_pointer = 0;
            while(Wire.available())    // slave may send less than requested
            {
                SLAVE_RX_DATA[slave_rx_pointer][i] = Wire.read();    // receive a byte as character
                slave_rx_pointer++;        // print the character
            }
            slave_rx_pointer = 0;
        }

        READY_TO_RECEIVE_FROM_SLAVE = 0;
        SLAVE_DATA_RECEIVED = 1;
    }
}