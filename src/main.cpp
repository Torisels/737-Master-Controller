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
#define FLAG_TRANSMIT_TO_PC 0xCD
#define FLAG_END_TRANSMIT_TO_PC 0xCE




#define SLAVE_SEND_BF_SIZE 32
#define SLAVE_RX_BF_SIZE 32

#define NUMBER_OF_SLAVES 4
#define FLAG_NEW_SLAVE 0xDD


#define BYTES_TO_RECEIVE_FROM_SLAVE 24

uint8_t FLAGS = 0;

#define READY_TO_TRANSMIT_TO_PC (1<<0)
#define READY_TO_TRANSMIT_TO_SLAVE  (1<<1)
#define READY_TO_RECEIVE_FROM_SLAVE (1<<2)
#define SLAVE_DATA_RECEIVED (1<<3)

uint8_t SLAVE_SEND_DATA[SLAVE_SEND_BF_SIZE][NUMBER_OF_SLAVES] = {};
uint8_t SLAVE_RX_DATA[NUMBER_OF_SLAVES][SLAVE_RX_BF_SIZE] = {};

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
            FLAGS |= READY_TO_TRANSMIT_TO_PC;
            break;
        case FLAG_TRANSMIT_TO_PC:

            READY_TO_TRANSMIT_TO_SLAVE = 1;
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

    }


    if(FLAGS & READY_TO_TRANSMIT_TO_PC)//transmit data to PC = normal mode
    {
        uint8_t t_buffer[32] = {FLAG_TRANSMIT_TO_PC};
        uint8_t counter = 1;
        for(int i=0;i<NUMBER_OF_SLAVES;i++)
        {
            t_buffer[counter] = FLAG_NEW_SLAVE;
            counter++;
           for(int j=0;j< sizeof(SLAVE_RX_DATA[i]);j++)
           {
               t_buffer[counter] = SLAVE_RX_DATA[i][j];
               counter++;
           }
        }
        t_buffer[counter] =  FLAG_END_TRANSMIT_TO_PC;
        //WRITE ROUTINE
        Serial.write(t_buffer, sizeof(t_buffer));
        FLAGS &= ~READY_TO_TRANSMIT_TO_PC;
    }
    if(FLAGS & READY_TO_TRANSMIT_TO_SLAVE)//Normal, no setup routine
    {
        for(int i=0;i<NUMBER_OF_SLAVES;i++)
        {
            Wire.beginTransmission(i);
            Wire.write(SLAVE_SEND_DATA[i],SLAVE_SEND_BF_SIZE);
            Wire.endTransmission();
        }
        FLAGS &= ~READY_TO_TRANSMIT_TO_SLAVE;
        FLAGS |= READY_TO_RECEIVE_FROM_SLAVE;
    }
    if(FLAGS & READY_TO_RECEIVE_FROM_SLAVE)
    {
        //rx from slave
        for(int i=0;i<NUMBER_OF_SLAVES;i++)
        {
            Wire.requestFrom(i,BYTES_TO_RECEIVE_FROM_SLAVE);
            uint8_t slave_rx_pointer = 0;
            while(Wire.available())
            {
                SLAVE_RX_DATA[i][slave_rx_pointer] = Wire.read();
                slave_rx_pointer++;
            }
        }

        FLAGS &= READY_TO_RECEIVE_FROM_SLAVE;
        FLAGS |=  SLAVE_DATA_RECEIVED;
    }
}