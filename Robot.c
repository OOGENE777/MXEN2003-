//include this .c file's header file
#include "Robot.h"
//static function prototypes, functions only called in this file

//file scope variables
static char serial_string[200] = {0};
static int16_t lm = 0;
static int16_t rm = 0;
static int16_t rc = 0;
static int16_t fc = 0;
int main(void)
{
    // initialisation
    serial0_init();     // terminal communication with PC
    serial2_init();     // microcontroller communication to/from another Arduino 
                // or loopback communication to same Arduino
    
    
    milliseconds_init();
        cli();
        DDRA |= (1<<DDA0)|(1<<DDA1)|(1<<DDA2)|(1<<DDA3); //put A0-A3 into low impedance output mode
        DDRB = 255;
        TCCR1A = 0;
        TCCR1B = 0;
        TCCR1A |= (1<<COM1B1)|(1<<COM1A1)|(1<<WGM11);
        TCCR1B |= (1<<CS11)|(1<<WGM13); 
        ICR1 = 2500;   
        //OCR1A = 1200;   
        sei();



    
    uint8_t sendDataByte1=0, sendDataByte2=0;       // data bytes sent
    uint8_t recvDataByte1=0, recvDataByte2=0, recvDataByte3=0, recvDataByte4=0;         // data bytes received
    
    uint32_t current_ms=0, last_send_ms=0;          // used for timing the serial send
    
    uint8_t serial_byte_in=0, serial_fsm_state=0;   // used in the serial receive section
    
    while(1)
    {
        current_ms = milliseconds_now();
        //sending section
        if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
        {   
            // this is just incrementing variables to send for testing purposes
            // you will put the code here that puts the message you want to send into sendDataByte1 and sendDataByte2
            sendDataByte1 = 5;
            sendDataByte2+=2;
            if (sendDataByte1>253)
                sendDataByte1 = 0;
            if (sendDataByte2>253)
                sendDataByte2 = 0;
            
            // you can add additional bytes to send in the message, 
            //but make sure the receiving code is expecting the right number of bytes
            
            last_send_ms = current_ms;
            serial2_write_byte(0xFF);       //send start byte = 255
            serial2_write_byte(sendDataByte1);  //send first data byte: must be scaled to the range 0-253
            serial2_write_byte(sendDataByte2);  //send second parameter: must be scaled to the range 0-253
            serial2_write_byte(0xFE);       //send stop byte = 254
        }
        //receiving section
        if(UCSR2A&(1<<RXC2)) //if new serial byte has arrived: refer to page 238 of datasheet. Single bit flag indicates a new byte is available
        {
            serial_byte_in = UDR2; //move serial byte into variable
            
            switch(serial_fsm_state) //switch by the current state
            {
                case 0:
                //do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
                break;
                case 1: //waiting for first parameter
                    recvDataByte1 = serial_byte_in;
                    serial_fsm_state++;
                break;
                case 2: //waiting for second parameter
                    recvDataByte2 = serial_byte_in;
                    serial_fsm_state++;
                break;
                case 3: //waiting for third parameter
                    recvDataByte3 = serial_byte_in;
                    serial_fsm_state++;
                break;

                case 4: //waiting for fourth parameter
                    recvDataByte4 = serial_byte_in;
                    serial_fsm_state++;
                break;

                case 5: //waiting for stop byte
                    if(serial_byte_in == 0xFE) //stop byte
                    {
                        // now that the stop byte has been received, we can process the whole message
                        // the code in this section will implement the result of your message
                        sprintf(serial_string, "received: 1:%4d, 2:%4d, 3:%4d, 4:%d  \n", recvDataByte1, recvDataByte2,recvDataByte3,recvDataByte4);
                        serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received                
                    } 
                    // if the stop byte is not received, there is an error, so no commands are implemented
                    serial_fsm_state = 0; //do nothing next time except check for start byte (below)
                break;
            }
            if(serial_byte_in == 0xFF) //if start byte is received, we go back to expecting the first data byte
            {
                serial_fsm_state=1;
            }
        }   

        
        fc = recvDataByte1;
        rc = recvDataByte2;
        lm = fc + rc - 253;
        rm = fc - rc;



        OCR1A = (int32_t)abs(lm)*2500/126; //lm speed from magnitude of lm
        OCR1B = (int32_t)abs(rm)*2500/126; //lm speed from magnitude of rm

        if(lm>=0) //if lm is positive
        {
        //set direction forwards
        PORTA |= (1<<PA0);
        PORTA &= ~(1<<PA1);
        }
        else
        {
        //set direction reverse
        PORTA &= ~(1<<PA0);
        PORTA |= (1<<PA1);
        }
        if(rm>=0) //if rm is positive
        {
        //set direction forwards
        PORTA |= (1<<PA2);
        PORTA &= ~(1<<PA3);
        }
        else
        {
        //set direction reverse
        PORTA &= ~(1<<PA2);
        PORTA |= (1<<PA3);
        }



    }
    return(1);
} //end main
ISR(TIMER1_CAPT_vect)
{
}
