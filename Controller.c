//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"



#include "adc.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include "hd44780.h"
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>



//file scope variables
static char serial_string[32] = {};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false;
int x, JSR_Vert, JSR_Horz, JSL_Vert, JSL_Horz, Range_Read;



// Servo Control Timer
uint32_t now1 = 0;
uint32_t now2 = 0;







int main(void)
{
	// initialisation
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
	// or loopback communication to same Arduino

  // Servo BUTTON
  DDRD = 0;
  DDRD &= ~(1<<PD0);
  PORTD |= (1<<PD0);
  EICRA |= (1<<ISC01);
  EICRA |= (1<<ISC00);
  EIMSK |= (1<<INT0);
  

  // SERVO TIMER
cli();
  DDRB = 255;
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1)|(1<< WGM11); // Clear on up-count, set on down-count
  TCCR1B |= (1 << CS10)|(1<<WGM13); // 64 prescaler = 488Hz base frequency
  TCCR1B |= (1 << CS12); 
  ICR1= 20000;
  sei();

	
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// data bytes sent
	
	uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send
	
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	milliseconds_init();
	sei();
  adc_init();
  lcd_init();


  //Pin assignments
  
  DDRF = 0;
  PORTF |= (1<<PF0);
  PORTF |= (1<<PF1);
  
  DDRK = 0;
  PORTK |= (1<<PK6);
  PORTK |= (1<<PK7);

  DDRH  = 0xFF;
  PINH |= (1<<PH1);
  PINH |= (1<<PH0);






	
	while(1)
	{
    JSR_Vert = adc_read(0)/4;
    JSR_Horz = adc_read(1)/4;
    JSL_Vert = adc_read(14)/4;
    JSL_Horz = adc_read(15)/4;

		current_ms = milliseconds_now();
		//sending section
		if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
		{
			// this is just incrementing variables to send for testing purposes
			// you will put the code here that puts the message you want to send into sendDataByte1 and sendDataByte2
      if(JSR_Vert>253){
        JSR_Vert = 253;
      }
			sendDataByte1 = JSR_Vert;
			
		if(JSR_Horz>253){
        JSR_Horz = 253;
      }
			sendDataByte2 = JSR_Horz;
			
			
  if(JSL_Vert>253){
        JSL_Vert = 253;
      }
			sendDataByte3 = JSL_Vert;
			
		if(JSL_Horz>253){
        JSL_Horz = 253;
      }
			sendDataByte4 = JSL_Horz;
			//You should replace the above data byte code with your own definitions
			//or calculations of what should be sent in the bytes
			
			// you can add additional bytes to send in the message,
			//but make sure the receiving code is expecting the right number of bytes
			
			last_send_ms = current_ms;
			serial2_write_byte(0xFF); 		//send start byte = 255
			serial2_write_byte(sendDataByte1); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte2); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte3); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte4); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(0xFE); 		//send stop byte = 25


//LCD Stuff
      lcd_clrscr();
      lcd_home();
      //lcd_puts("range is:");
      //lcd_goto(0x40);
      lcd_puts(serial_string); //Print string to LCD first line
		//lcd_goto( 0x40 );     //Put cursor to first character on second line
 
      


      
		}

		//if a new byte has been received
		if(new_message_received_flag) 
		{
			// now that a full message has been received, we can process the whole message
			// the code in this section will implement the result of your message
			sprintf(serial_string, "%4d \n", dataByte1);
			serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received



			new_message_received_flag=false;	// set the flag back to false
		}




















	}
	return(1);
} //end main


ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0, recvByte4=0;		// data bytes received
	static uint8_t serial_fsm_state=0;									// used in the serial receive ISR
	uint8_t	serial_byte_in = UDR2; //move serial byte into variable
	
	switch(serial_fsm_state) //switch by the current state
	{
		case 0:
		//do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
		break;
		case 1: //waiting for first parameter
		recvByte1 = serial_byte_in;
		serial_fsm_state++;
		break;
		/*case 2: //waiting for second parameter
		recvByte2 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 3: //waiting for second parameter
		recvByte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for second parameter
		recvByte4 = serial_byte_in;
		serial_fsm_state++;
		break;*/
		case 5: //waiting for stop byte

    
		if(serial_byte_in == 0xFE) //stop byte
		{
			// now that the stop byte has been received, set a flag so that the
			// main loop can execute the results of the message
			dataByte1 = recvByte1;
			//dataByte2 = recvByte2;
			//dataByte3 = recvByte3;
			//dataByte4 = recvByte4;
			
			new_message_received_flag=true;
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


//int main(void)
//{
  //variable declarations
	//char lcd_string[33] = {0}; //declare and initialise string for LCD

	//initialisation section, runs once
//	adc_init(); //initialse ADC
//	lcd_init(); //initialise 

	//_delay_ms(20);
	//uint16_t variableToPrint;

  //int duty=50; // This number will result in a wave duty cycle
// of about 50%, since 256/2=128
  //TCCR1A = 0; TCCR1B = 0;
  //TCCR1B |= (1 << WGM13); // Phase correct, 8 bit PWM
//  TCCR1A |= (1 << COM1A1)|(1<< WGM11); // Clear on up-count, set on down-count
  //TCCR1B |= (1 << CS10); // 64 prescaler = 488Hz base frequency
//  TCCR1B |= (1 << CS12); 
 // ICR1= 625;
  //OCR1A = duty; // comparison value
/* 
	
  DDRA = 0xFF;
  DDRD &= ~(1<<PD0);
  PORTD |= (1<<PD0);
  EICRA|= (1<<ISC01);
  EICRA |= (1<<ISC00);
  EIMSK |= (1<<INT0);
  sei();

  PORTA = 0;
  */

	//main loop
	/*while(1)
	{	
    int x, JSR_Vert, JSR_Horz, JSL_Vert, JSL_Horz;
    JSR_Vert = adc_read(0);
    JSR_Horz = adc_read(1);
    JSR_Vert = adc_read(2);
    JSR_Horz = adc_read(3);
















    //adcValV= adc_read(0);
    
        //500 millisecond delay
    //int n =(adcValV>>7);
   //PORTA = (1<<n) ; // note here PA3 is just an alias for the number 3
                        // this line is equivalent to PORTA = PORTA | 0b00001000   which writes a HIGH to pin 3 of PORTA
    


    


  
    //lcd_goto(0);      //Put cursor at position 0
    //lcd_home();       // same as lcd_goto(0);
		
    //_delay_ms(500);
    //lcd_puts( " the time is:" ); //Print string to LCD first line
		//lcd_goto( 0x40 );     //Put cursor to first character on second line

    
    //sprintf( lcd_string , "%u:%u:%u" , min, sec, mil); 
    //print to string, %u special character to be replaced by variables in later arguments
   
   
    //lcd_puts( lcd_string ); //Print string to LCD second line, same as first line
    //%u for unsigned integers, %i,%d for signed integers
    //%lu for long unsigned ...
  
    
    
    /*lcd_clrscr();     //Clear everything on LCD
		variableToPrint = 111;
		sprintf( lcd_string , "%u" , variableToPrint );
		lcd_goto(0);     
    lcd_puts( lcd_string ); //Screen shows 111
		variableToPrint = 3;
		sprintf( lcd_string , "%u" , variableToPrint );
		lcd_puts( lcd_string ); // screen shows 311
		sprintf( lcd_string , "%3u" , variableToPrint );
		lcd_puts( lcd_string ); // screen now has 3
		//loop code goes here	
  


    




     // this line is equivalent to PORTA = PORTA & (0b11110111)  which writes a HIGH to pin 3 of PORTA
    
   
    
    _delay_ms(500);     //500 millisecond delay
    PORTA |= (1<<PA3);  // note here PA3 is just an alias for the number 3
                        // this line is equivalent to PORTA = PORTA | 0b00001000   which writes a HIGH to pin 3 of PORTA


    _delay_ms(500); 
    PORTA &= ~(1<<PA3); // this line is equivalent to PORTA = PORTA & (0b11110111)  which writes a HIGH to pin 3 of PORTA


    _delay_ms(500);     //500 millisecond delay
    PORTA |= (1<<PA4);  // note here PA3 is just an alias for the number 3
                        // this line is equivalent to PORTA = PORTA | 0b00010000   which writes a HIGH to pin 3 of PORTA


    _delay_ms(500); 
    PORTA &= ~(1<<PA4); // this line is equivalent to PORTA = PORTA & (0b11110111)  which writes a HIGH to pin 3 of PORTA


    _delay_ms(500);     //500 millisecond delay
    PORTA |= (1<<PA5);  // note here PA3 is just an alias for the number 3
                        // this line is equivalent to PORTA = PORTA | 0b00001000   which writes a HIGH to pin 3 of PORTA


    _delay_ms(500); 
    PORTA &= ~(1<<PA5); // this line is equivalent to PORTA = PORTA & (0b11110111)  which writes a HIGH to pin 3 of PORTA


    _delay_ms(500);     //500 millisecond delay
    PORTA |= (1<<PA6);  // note here PA3 is just an alias for the number 3
                        // this line is equivalent to PORTA = PORTA | 0b00001000   which writes a HIGH to pin 3 of PORTA


    _delay_ms(500); 
    PORTA &= ~(1<<PA6); // this line is equivalent to PORTA = PORTA & (0b11110111)  which writes a HIGH to pin 3 of PORTA

    _delay_ms(500);     //500 millisecond delay
    PORTA |= (1<<PA7);  // note here PA3 is just an alias for the number 3
                        // this line is equivalent to PORTA = PORTA | 0b00001000   which writes a HIGH to pin 3 of PORTA


    _delay_ms(500); 
    PORTA &= ~(1<<PA7); // this line is equivalent to PORTA = PORTA & (0b11110111)  which writes a HIGH to pin 3 of PORTA
*/
