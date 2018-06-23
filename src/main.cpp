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
#define FLAG_SLAVE_SETUP_SUCCESS 0xCF
#define FLAG_SLAVE_SETUP_FAIL 0xCE

#define FLAG_PC_NORMAL_RX_MODE 0xAA
#define FLAG_PC_SETUP_RX_MODE 0xAB

#define FLAG_7_SEG_ROUTINE 0xDA
#define GRB_ROUTNE 0xDB
#define FLAG_SLAVE_MASTER_SETUP 0xFA

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
#define PC_DATA_SENT (1<<4)
#define SETUP_ROUTINE (1<<5)


uint8_t SLAVE_SEND_DATA[SLAVE_SEND_BF_SIZE][NUMBER_OF_SLAVES] = {};
uint8_t SLAVE_RX_DATA[NUMBER_OF_SLAVES][SLAVE_RX_BF_SIZE] = {};


uint8_t SERIAL_RX_BUFFER[64] = {0};

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
//        case FLAG_TRANSMIT_TO_PC:
//
//            READY_TO_TRANSMIT_TO_SLAVE = 1;
//            break;




        default:
            break;
    }


}









void setup()
{
    //DDRA |= 1 << PA0;
   // output_grb_strip(arr, sizeof(arr));
    Serial.begin(115200);
    pinMode(LED_BUILTIN,OUTPUT);
}







void loop()
{
    if(Serial.available()>0)
    {
        uint8_t F_FLAG = Serial.read();
        if(F_FLAG == FLAG_PC_NORMAL_RX_MODE)
        {

        digitalWrite(LED_BUILTIN,HIGH);
        uint8_t bubu[3] = {0xDD,0xDB,0xDC};
        Serial.write(bubu,3);
//Serial.println("XDD");







//            FLAGS |= READY_TO_TRANSMIT_TO_PC;
//            FLAGS |= READY_TO_TRANSMIT_TO_SLAVE;
        }
        /* SETUP ROUTINE
         * 1. FLAG_SETUP_RX_MODE
         *
         * SLAVES
         * {2. FLAG_NEW_SLAVE
         * 3. SLAVE_ID
         * 4. PORTA-D,PINA-D
         * 5. USE_ANALOG
         * 6. (ANALOG_CHANNELS_ACTIVE)
         * 7. (ANALOG_BIT_MASK)}
         * 8. FLAG SETUP FINISHED
         * */
        else if(F_FLAG == FLAG_PC_SETUP_RX_MODE)
        {
            int slaves_counter = 0;
            int slave_id = -1;

            if(Serial.read()==FLAG_NEW_SLAVE)//this is for first slave
            {
                for(int i=0;i<NUMBER_OF_SLAVES;i++)
                {
                    uint8_t slave_buffer[SLAVE_SEND_BF_SIZE] = {FLAG_SERIAL_MASTER_SETUP};
                    int buffer_counter = 1;

                    if(Serial.available()<10)
                        break;

                    slave_id = Serial.read();
                    if(slave_id!=i)
                        break;
                    uint8_t current;
                    while((current = Serial.read())!=FLAG_NEW_SLAVE)
                    {
                        slave_buffer[buffer_counter] = current;
                        buffer_counter++;
                    }
                    /*MASTER -> SLAVE */
                    Wire.beginTransmission(i);
                    Wire.write(slave_buffer,buffer_counter+1);
                    Wire.endTransmission();
                    /*ASK FOR CONFIRMATION*/
                    Wire.requestFrom(i,1);
                    uint8_t rx = 0;
                    if(Wire.available())
                    {
                        rx = Wire.read();
                    }
                    /*INFORM PC*/
                    if(rx==FLAG_SLAVE_SETUP_SUCCESS)
                    {
                        Serial.write(FLAG_SLAVE_SETUP_SUCCESS);
                        slaves_counter++;
                    }
                    else{
                        Serial.write(FLAG_SLAVE_SETUP_FAIL);
                    }

                }
            }
            if(slaves_counter==NUMBER_OF_SLAVES-1)
            {
                FLAGS |= SETUP_ROUTINE; //setup success
            }
            FLAGS|=READY_TO_TRANSMIT_TO_PC;
        }
        else if(F_FLAG == FLAG_7_SEG_ROUTINE)
        {
            //implement max7219 routine
        }
        else if(F_FLAG == GRB_ROUTNE)
        {
            //implement grb routine
        }
    }


    if(FLAGS & READY_TO_TRANSMIT_TO_PC)//transmit data to PC = normal mode
    {
        uint8_t t_buffer[32] = {FLAG_TRANSMIT_TO_PC};
        int counter = 1;
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
        FLAGS |= PC_DATA_SENT;
    }
//    if(FLAGS & SETUP_ROUTINE)//setup routine
//    {
//        for(int i=0;i<NUMBER_OF_SLAVES;i++)
//        {
//            Wire.beginTransmission(i);
//            Wire.write(SLAVE_SEND_DATA[i],SLAVE_SEND_BF_SIZE);
//            Wire.endTransmission();
//        }
//
//        for(int i=0;i<NUMBER_OF_SLAVES;i++)
//        {
//            Wire.requestFrom(i,1);
//            while(Wire.available())
//            {
//              if(Wire.read() == FLAG_SLAVE_SETUP_SUCCESS)
//              {
//                  FLAGS &= ~SETUP_ROUTINE;
//                  FLAGS |= READY_TO_TRANSMIT_TO_PC;
//              }
//            }
//        }
//    }
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
    if(FLAGS & READY_TO_RECEIVE_FROM_SLAVE && FLAGS&PC_DATA_SENT)
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

        FLAGS &= ~READY_TO_RECEIVE_FROM_SLAVE;
        FLAGS |= SLAVE_DATA_RECEIVED;
        FLAGS &= ~PC_DATA_SENT;
    }
}