#define F_CPU 14745600 //frequency of CPU
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.h"//for including LCD related functions

//Sensor values after ADC conversion
unsigned char White_1, White_2, White_3; //White line sensor values
                 //1 is extreme left, 2 is middle and 3 is extreme right
unsigned char Sharp_left, Sharp_right; //Sharp IR sensor values
                //Sensor on left side is connected to sharp connector 1 on robot
				//Sensor on right side is connected to sharp connector 5 on robot
unsigned char left_ir, right_ir; //Analog IR proximity sensor values
                //Sensor on left side is connected to IR connector 2
				//Sensor on right side is connected to IR connector 5
				
				
//Arena monitoring variables
unsigned int count = 0; //total no of right turn junctions crossed (with the exception of the starting junction)
unsigned int weed_full = 0;//a boolean variable to check if container contains uprooted weeds.
       //If 0, no weed present. If 1, at least one uprooted weed present in container. 
	  
//Thresholds for sensor values. These thresholds are experimentally calibrated and may differ from
//time to time.
unsigned int tw_1 = 60;//Threshold for white line sensor 1
unsigned int tw_2 = 60;//Threshold for white line sensor 2
unsigned int tw_3 = 60;//Threshold for white line sensor 3
unsigned int ts_l = 60;//Threshold for left sharp sensor
unsigned int ts_r = 60;//Threshold for right sharp sensor
unsigned int tr_l = 150;//Threshold for left analog IR sensor
unsigned int tr_r = 150;//Threshold for right analog IR sensor

//Variables for black line navigation
float pGain = 60;//gain factor for proportional control
float control; //control variable
float s; //average value for white line sensors
unsigned int white_value;//the contents of all 3 white line sensor values
                       //arranged sequentially as a single integer variable

//Position encoder usage variables
unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning



//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}

//Function to initialize Buzzer
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;	//Setting PORTC 3 logic low to turnoff buzzer
}

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
	DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
	PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}


//Function to initialize ports
void port_init()
{
	motion_pin_config();
	left_encoder_pin_config();
	right_encoder_pin_config();
	lcd_port_config(); 
	adc_pin_config();
	buzzer_pin_config();
	servo1_pin_config();
	servo2_pin_config();
	servo3_pin_config();
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz ; needed for servo operation
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256 PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

//Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref = 5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Buzzer functions
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}



void init_devices()
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	adc_init(); //Initialize the ADC
	timer1_init(); //Initialize timer 1 for servo control
	timer5_init(); //Initialize timer 5 for locomotion control
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   // Enables the global interrupt
}

//These functions control the servo motor rotation.

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}

//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
	OCR1CH = 0x03;
	OCR1CL = 0xFF; //Servo 3 off
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void)
{
  motion_set(0x00);
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		lcd_print(1, 1, ShaftCountLeft, 3);
		lcd_print(1, 5, ShaftCountRight, 3);
		lcd_print(1, 9, ReqdShaftCountInt, 3);
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop action
}

//Function used for moving robot forward by specified distance
void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		lcd_print(1, 1, ShaftCountLeft, 3);
		lcd_print(1, 5, ShaftCountRight, 3);
		lcd_print(1, 9, ReqdShaftCountInt, 3);
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop action
}


void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}

void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left_2(); //Turn reverse soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right_2();  //Turn reverse soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

float readSensors()
{
	float avgSensors = 0.0;//weighted mean value of the white line sensors
	
	//Thresholding all Sharp and analog IR sensor values
	Sharp_left = ADC_Conversion(9);
	Sharp_right = ADC_Conversion(13);
	left_ir = ADC_Conversion(5);
	right_ir = ADC_Conversion(8);

	//Thresholding white line sensor values
	White_1 = (ADC_Conversion(3) >= tw_1)?1:0;
	White_2 = (ADC_Conversion(2) >= tw_2)?1:0;
	White_3 = (ADC_Conversion(1) >= tw_3)?1:0;
		
	//calculation of weighted mean white line sensor value
	white_value = White_1*100 + White_2*10 + White_3;
	float sum = White_1 + White_2 + White_3;
	if(sum == 0.0) //prevent division by zero
	{
		return 0.0;
	}
	
	avgSensors = (White_1 * 1 + White_2 * 2 + White_3 * 3)/sum;
	return avgSensors;
}

//Function to generate control variable using proportional control algorithm
float Pcontrol(float cur_value, float req_value) //Here the average sensor value is the cur_value
//and 2.0 is the required value
{
	float con = 0.0;
	float error;
	error = req_value - cur_value;
	con = (pGain * error);
	if(con < 0) con *= -1; //To extract only magnitude of control without sign
	return con;
}

//Function to uproot weed, Servo connector 1 is used to control the arm,
//Servo connector 2 is used to control the gripper
void uproot_weed()
{
	//Initially bringing both servos to their zero position
	servo_1(0);
	servo_2(0);
	
	//moving arm downward
	for( int j = 0; j <= 178; j++)
	{
		servo_1(j);
		_delay_ms(3);
	}
	
	//gripping weed
	for(int k = 0; k < 180; k++)
	{
		servo_2(k);
		_delay_ms(5);
	}
	
	_delay_ms(10); //delay to allow weed to get firmly gripped
	
	//moving arm upward	
	for(int i = 178; i >= 0; i--)
	{
		servo_1(i);
		_delay_ms(3);
	}
	
	//releasing weed
	for(int l = 180; l >= 0; l--)
	{
		servo_2(l);
		_delay_ms(2);
	}
	
	weed_full = 1;//the container now has at least 1 weed, so deposition routine must 
	              //work at the next junction.
	servo_1_free();
	servo_2_free();
}

//Function to open the container to drop weeds in deposition zone
//Servo 3 connector controls the servo used for this purpose
void deposit_weed()
{
	servo_3(0); //Initial position of servo brought to zero
	back_mm(60);//move robot 60 cm back into deposition zone
	
	//open container to let weeds fall
	for(int m = 0; m <= 140; m++)
	{
		servo_3(m);
		_delay_ms(3);
	}
	
	forward_mm(60); //move robot 80 cm forward to land up again over black line
	
	//close container
	for(int n = 140; n >= 0; n--)
	{
		servo_3(n);
		_delay_ms(3);
	}
	
	weed_full = 0; //weeds have been deposited.
	servo_3_free();
}

//Main Function
int main(void)
{
	init_devices(); //Initialize all devices and ports
	lcd_init();
	lcd_set_4bit();
		
	while(1)
	{
		s = readSensors(); //read all sensor values and return the mean white line sensor value
				
		//Generating control variable for turning
		control = Pcontrol(s, 2.0);
		
		//Printing sensor values on LCD
		lcd_print(1,1,left_ir,3);
		lcd_print(1,13,right_ir,3);
		lcd_print(2,1,Sharp_left,3);
		lcd_print(2,13,Sharp_right,3);
		lcd_print(2, 5, white_value, 3);

		forward();
		velocity(190,190);
		
		if(white_value == 11 || white_value == 111 || white_value == 110)
		{
				buzzer_on();
				_delay_ms(10); //indication that junction has been detected
				buzzer_off();
				forward_mm(50);//to coincide wheel axis with turning point
				velocity(200,200);
				count++;
				right_degrees(84);// experimentally found that 84 in software led to approximate 90 degree turn physically
				_delay_ms(500);
				
					
				if(weed_full == 1)
				{
					deposit_weed();
				}
				if(count == 3)
				{
					stop();
					buzzer_on();
					_delay_ms(6000);
					buzzer_off();
					forward_mm(60);
					break; //terminate application
				}
		}
		
		if(white_value != 10)
		{
			if(s < 2.0 && s > 0.0)//black line to left 
			{
				
					s = readSensors();
					
					if(white_value == 11 || white_value == 111 || white_value == 110)
					{
						buzzer_on();
						_delay_ms(50);
						buzzer_off();
						forward_mm(50);//to coincide wheel axis with turning point
						velocity(200,200);
						count++;
						right_degrees(84-count);
						_delay_ms(500);
						
								
						if(weed_full == 1)
						{
							deposit_weed();
						}
						if(count == 3)
						{
							stop();
							buzzer_on();
							_delay_ms(6000);
							buzzer_off();
							forward_mm(60);
							break; //terminate application
						}
					}
					velocity(180,200);
			}
			else if(s > 2.0) //black line to right
			{
				
					s = readSensors();
					
					if(white_value == 11 || white_value == 111 || white_value == 110)
					{
						buzzer_on();
						_delay_ms(50);
						buzzer_off();
						forward_mm(50);//to coincide wheel axis with turning point
						velocity(200,200);
						count++;
						right_degrees(84);
						_delay_ms(500);
						
						
						if(weed_full == 1)
						{
							deposit_weed();
						}
						if(count == 3)
						{
							stop();
							buzzer_on();
							_delay_ms(6000);
							buzzer_off();
							forward_mm(60);
							break; //terminate application
						}
					}
					velocity(200,180);
			}
		}	
		
		
		if((Sharp_left <= ts_l) && ((left_ir < tr_l)) && (count!=1))
		{
			
			stop();
			buzzer_on();
			_delay_ms(50);
			buzzer_off();
			velocity(200,200);
			left_degrees(84);
			uproot_weed();
			weed_full=1;
			right_degrees(84);
			_delay_ms(500);
		}
		
		if((Sharp_right <= ts_r) && ( (right_ir < tr_r) && (count!=1)))
		{
			stop();
			buzzer_on();
			_delay_ms(50);
			buzzer_off();
			velocity(200,200);
			right_degrees(84);
			uproot_weed();
			weed_full=1;
			left_degrees(84);
			_delay_ms(500);
		}
	}
}