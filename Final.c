//
//  CCexpAvr.c
//  Line following Robot
//
//  Created by Ayan Sengupta and Sarthak Sharma on 25/07/16.


#include<avr/io.h>
#include <avr/interrupt.h>
#include<util/delay.h>
#include <avr/delay.h>


///////////////////////////////////////////////////////////////////////////
////////////////            LCD          //////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#define RS 0
#define RW 1
#define EN 2
#define lcd_port PORTC

#define sbit(reg,bit)	reg |= (1<<bit)
#define cbit(reg,bit)	reg &= ~(1<<bit)

void init_ports();
void lcd_reset_4bit();
void lcd_init();
void lcd_wr_command(unsigned char);
void lcd_wr_char(char);
void lcd_home();
void lcd_cursor(char, char);
void lcd_print(char, char, unsigned int, int);
void lcd_string(char*);

unsigned int temp;
unsigned int unit;
unsigned int tens;
unsigned int hundred;
unsigned int thousand;
unsigned int million;

int i;


//Function to Reset LCD
void lcd_set_4bit()
{
	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3 in the upper nibble
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3 in the upper nibble
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3 in the upper nibble
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x20;				//Sending 2 in the upper nibble to initialize LCD 4-bit mode
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//delay
	cbit(lcd_port,EN);				//Clear Enable Pin
}

//Function to Initialize LCD
void lcd_init()
{
	_delay_ms(1);

	lcd_wr_command(0x28); //4-bit mode and 5x8 dot character font
	lcd_wr_command(0x01); //Clear LCD display
	lcd_wr_command(0x06); //Auto increment cursor position
	lcd_wr_command(0x0E); //Turn on LCD and cursor
	lcd_wr_command(0x80); //Set cursor position
}


//Function to write command on LCD
void lcd_wr_command(unsigned char cmd)
{
	unsigned char temp;
	temp = cmd;
	temp = temp & 0xF0;
	lcd_port &= 0x0F;
	lcd_port |= temp;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);

	cmd = cmd & 0x0F;
	cmd = cmd<<4;
	lcd_port &= 0x0F;
	lcd_port |= cmd;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}

//Function to write data on LCD
void lcd_wr_char(char letter)
{
	char temp;
	temp = letter;
	temp = (temp & 0xF0);
	lcd_port &= 0x0F;
	lcd_port |= temp;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);

	letter = letter & 0x0F;
	letter = letter<<4;
	lcd_port &= 0x0F;
	lcd_port |= letter;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}


void lcd_home()
{
	lcd_wr_command(0x80);
}


//Function to Print String on LCD
void lcd_string(char *str)
{
	while(*str != '\0')
	{
		lcd_wr_char(*str);
		str++;
	}
}

//Position the LCD cursor at "row", "column"

void lcd_cursor (char row, char column)
{
	switch (row) {
		case 1: lcd_wr_command (0x80 + column - 1); break;
		case 2: lcd_wr_command (0xc0 + column - 1); break;
		case 3: lcd_wr_command (0x94 + column - 1); break;
		case 4: lcd_wr_command (0xd4 + column - 1); break;
		default: break;
	}
}

//Function to print any input value up to the desired digit on LCD
void lcd_print (char row, char coloumn, unsigned int value, int digits)
{
	unsigned char flag=0;
	if(row==0||coloumn==0)
	{
		lcd_home();
	}
	else
	{
		lcd_cursor(row,coloumn);
	}
	if(digits==5 || flag==1)
	{
		million=value/10000+48;
		lcd_wr_char(million);
		flag=1;
	}
	if(digits==4 || flag==1)
	{
		temp = value/1000;
		thousand = temp%10 + 48;
		lcd_wr_char(thousand);
		flag=1;
	}
	if(digits==3 || flag==1)
	{
		temp = value/100;
		hundred = temp%10 + 48;
		lcd_wr_char(hundred);
		flag=1;
	}
	if(digits==2 || flag==1)
	{
		temp = value/10;
		tens = temp%10 + 48;
		lcd_wr_char(tens);
		flag=1;
	}
	if(digits==1 || flag==1)
	{
		unit = value%10 + 48;
		lcd_wr_char(unit);
	}
	if(digits>5)
	{
		lcd_wr_char('E');
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////



#define THRESHOLD  30
#define BASESPEED 80

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char Left_sensor = 0 ;
unsigned char Right_sensor = 0;
unsigned char Center_sensor = 0;


// Define pid parameters
float Kp = 120;
float Ki = 0.3;
float Kd = 50;
float error = 0, perror = 0, Iprev = 0;
float P, I, D, correction;


//PWM

void pwm_init(void)
{

    // fast PWM with TOP as 00FF
    // WGM13 WGM12 WGM11 WGM 10
    //   0     1     0     1

    TCCR1B = 0x00; //stop
    TCNT1H = 0xFF; //setup
    TCNT1L = 0x01;
    OCR1AH = 0x00;
    OCR1AL = 0xFF;
    OCR1BH = 0x00;
    OCR1BL = 0xFF;
    ICR1H  = 0x00;
    ICR1L  = 0xFF;
    TCCR1A = 0xA1;
    TCCR1B = 0x0D; //start Timer
}


// ALL PIN CONFIGS

void motion_pin_config (void)
{
    DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
    PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic0
    DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
    PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}





void lcd_port_config (void) {

    DDRC = DDRC | 0xF7;
    PORTC = PORTC & 0x80;

}


//All the ADC pins must be configured as input and floating

void adc_pin_config (void){

    DDRA = 0x00;   //set PORTF direction as input
    PORTA = 0x00;  //set PORTF pins floating

}


void port_init()
{
    motion_pin_config();
    lcd_port_config();
    adc_pin_config();
}

void adc_init() {
    ADCSRA = 0x00;
    ADMUX = 0x20; //Mux1 is high
    ACSR = 0x80;

    //ADEN = 1, ADPS2 = 1, ADPS1 = 1, ADPS0 = 0
    //ADC prescaler selection = 64

    ADCSRA = 0x86;
}

//ADC CONVERSION

unsigned char ADC_Conversion(unsigned char Ch)
{
    unsigned char a;
    Ch = Ch & 0x07;
    ADMUX= 0x20| Ch;
    ADCSRA = ADCSRA | 0x40;	//Set start conversion bit
    while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
    a=ADCH;
    ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
    return a;
}


void sensor_value(char row, char coloumn, char channel)
{
    ADC_Value = ADC_Conversion(channel);
    lcd_print(row, coloumn, ADC_Value, 3);

}




void motion_set (unsigned char Direction)
{
    unsigned char PortBRestore = 0;

    Direction &= 0x0F; 			// removing upper nibbel as it is not needed
    PortBRestore = PORTB; 			// reading the PORTB's original status
    PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
    PortBRestore |= Direction;
    PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void)         //both wheels forward
{
    motion_set(0x06);
}

void back (void)            //both wheels backward
{
    motion_set(0x09);
}

void left (void)            //Left wheel backward, Right wheel forward
{
    motion_set(0x05);
}

void right (void)           //Left wheel forward, Right wheel backward
{
    motion_set(0x0A);
}

void soft_left (void)       //Left wheel stationary, Right wheel forward
{
    motion_set(0x04);
}

void soft_right (void)      //Left wheel forward, Right wheel is stationary
{
    motion_set(0x02);
}

void soft_left_2 (void)     //Left wheel backward, right wheel stationary
{
    motion_set(0x01);
}

void soft_right_2 (void)    //Left wheel stationary, Right wheel backward
{
    motion_set(0x08);
}

void hard_stop (void)       //hard stop(stop suddenly)
{
    motion_set(0x00);
}

void soft_stop (void)       //soft stop(stops solowly)
{
    motion_set(0x0F);
}


//Function defining speed of motors

void speed(int l_motor,int r_motor){


			if (l_motor>250){
				l_motor = 250;
				}
			if (l_motor<0){
				l_motor = 0;
				}
			if (r_motor>250){
				r_motor = 250;
				}
			if (r_motor<0){
				r_motor = 0;
				}
    OCR1AH = 0x00;
    OCR1AL = l_motor;
    OCR1BH = 0x00;
    OCR1BL = r_motor;
}


void init_devices (void)
{
    cli();
    port_init();
    pwm_init();
    adc_init();
    sei();
}




// calculate error for pid implementation
void calc_error(char s1, char s2, char s3){

	float s;



    perror = error;




    s = (s1+s2+s3);
    error = ((1*s1 + 2*s2 +3*s3)/s);
	error = error - 1.9;
	//int e = (int)(error*10);
	}

//pid
void pid(void){


    P = error * Kp;

    Iprev += error;
    I = Iprev * Ki;
	D = 0;
   	D = Kd*( error - perror);

    correction = P + I + D;





}



// MAIN CODE
int main(void) {


	init_devices();

    lcd_init();
    lcd_set_4bit();
	int flag =0;


    while(1){



        Left_sensor = ADC_Conversion(3); //ADC3 for left sensor
        Right_sensor = ADC_Conversion(5); //ADC5 for right sensor
        Center_sensor = ADC_Conversion(4); //ADC4 for center sensor


		_delay_ms(5);
		calc_error(Left_sensor,Center_sensor,Right_sensor);
		pid();
		_delay_ms(5);





        sensor_value(1,1,3);		//Prints value of Left sensor
        sensor_value(1,5,4);		//Prints value of Center sensor
        sensor_value(1,9,5);		//Prints value of Right sensor
		_delay_ms(10);





       if (Center_sensor > THRESHOLD) {


        speed(255,255);
      	forward();
		flag =1;

		//	 _delay_ms(50);

        }

		if ((Center_sensor < THRESHOLD) && (Left_sensor < THRESHOLD) && (Right_sensor > THRESHOLD)){


            speed(50 + correction,50 + correction);
          	//forward();

          right();
		  flag =2;
			_delay_ms(150);


        }

		if ((Center_sensor < THRESHOLD) && (Left_sensor > THRESHOLD) && (Right_sensor < THRESHOLD)){


            speed(50 - correction,50 - correction);
            left();
           // forward();
		   flag = 3;
			_delay_ms(150);


        }

		if ((Center_sensor > THRESHOLD) && (Left_sensor > THRESHOLD) && (Right_sensor > THRESHOLD)){


            speed(200,200);

            forward();
			_delay_ms(1000);

			}


		if ((Center_sensor < THRESHOLD) && (Left_sensor < THRESHOLD) && (Right_sensor < THRESHOLD)){


           // speed(100,100);

           // back();
		//	_delay_ms(250);


			 if (flag ==2){
			 speed(50 + correction,50 + correction);
          	 right();
			 _delay_ms(250);

			}
			 else if (flag ==3){
			  speed(50 - correction,50 - correction);
              left();
			  _delay_ms(250);

			}
			else {

			 speed(100,100);
            back();
			_delay_ms(250);

			}

			}





    }
   // return 0;

}






