#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"

#define   THRESHOLD   50       // set the pots such that all three sensor 
                                      // calibrated to show its min value on LCD. 
                                      // i.e on LCD Sensor values are betwn 168 to 172
                    // on black line  
#define   VELOCITY_MAX  255
#define   VELOCITY_MIN  150
#define   VELOCITY_LOW  0
#define   ERROR_THRESH  10

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char left = 0;
unsigned char center = 0;
unsigned char right = 0;
unsigned char prev_left = 0;
unsigned char prev_right = 0;
unsigned char prev_center = 0;
float kp = 1;
float ki = 0;
float kd = 0;
float integrate = 0;
float prev = 0;
float error = 0;
float output = 0;
float diff = 0;


//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRA = 0x00;   //set PORTF direction as input
 PORTA = 0x00;  //set PORTF pins floating
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;    //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0;  //set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;    //Setting PD5 and PD4 pins as output for PWM generation
 PORTD = PORTD | 0x30;  //PD5 and PD4 pins are for velocity control using PWM
}

//Function to Initialize PORTS
void port_init()
{
 lcd_port_config();
 adc_pin_config();    
 motion_pin_config();
}

//TIMER1 initialize - prescale:64
// WGM: 5) PWM 8bit fast, TOP=0x00FF
// desired value: 450Hz
// actual value: 450.000Hz (0.0%)
void timer1_init(void)
{
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


//Function to Initialize ADC
void adc_init()
{
 ADCSRA = 0x00;
 ADMUX = 0x20;    //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;   //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 Ch = Ch & 0x07;        
 ADMUX= 0x20| Ch;       
 ADCSRA = ADCSRA | 0x40;  //Set start conversion bit
 while((ADCSRA&0x10)==0); //Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 return a;
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location. 
void print_sensor(char row, char coloumn,unsigned char channel)
{
 ADC_Value = ADC_Conversion(channel);
 lcd_print(row, coloumn, ADC_Value, 3);
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F;       // removing upper nibbel as it is not needed
 PortBRestore = PORTB;      // reading the PORTB's original status
 PortBRestore &= 0xF0;      // setting lower direction nibbel to 0
 PortBRestore |= Direction;   // adding lower nibbel for direction command and restoring the PORTB status
 PORTB = PortBRestore;      // setting the command to the port
}

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void backward (void)
{
  motion_set(0x09);
}
void stop (void) //hard stop
{
  motion_set(0x00);
}

void hard_left (void)            //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void hard_right (void)           //Left wheel forward, Right wheel backward
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

void soft_stop (void)       //soft stop(stops solowly)
{
  motion_set(0x0F);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
 OCR1AH = 0x00;
 OCR1AL = left_motor;
 OCR1BH = 0x00;
 OCR1BL = right_motor;
}

void init_devices (void)
{
  cli();          //Clears the global interrupts
  port_init();
  timer1_init();
  adc_init();
  sei();          //Enables the global interrupts
}

//Main Function
int main(void) {
  unsigned char flag ;

  init_devices();

  lcd_set_4bit();
  lcd_init();

  velocity(VELOCITY_MIN, VELOCITY_MIN);

  forward();
  int movement = 0;
  while(1) {
    left = ADC_Conversion(3);
    center = ADC_Conversion(4);
    right = ADC_Conversion(5);

    print_sensor(1,1,3);
    print_sensor(1,5,4);
    print_sensor(1,9,5);

    // Printing threshold values

    error = left - right;
    diff = error - prev;
    output = kp*error + ki*integrate + kd*diff;
    integrate += error;
    prev = error;
    lcd_cursor(2,8);
    lcd_wr_char('N');
    if (error > 0) {
      lcd_cursor(2,1);
      lcd_wr_char('+');
      lcd_print(2, 2, (int)output, 3);
    } else {
      lcd_cursor(2,1);
      lcd_wr_char('-');
      lcd_print(2, 2, -1*(int)output, 3);
    }

    if (left > 50 && right > 50 ) {
      velocity(255,255);
      forward();
    }
    else if (left < 50 && right < 50 && center < 50 
             && prev_left < 50 && prev_right < 50 && prev_center > 50) {
        velocity(255, 255);
        forward();
        lcd_cursor(2,8);
        lcd_wr_char('D');
        _delay_ms(2000);
    }
    else if (output > -50 && output < 50) {
      if (center > 50) {
        // Move forward
        velocity(255, 255);
        forward();
      } else {
        // out of bounds, move backward
        velocity(75, 75);
        backward();
      }
    }
    else if (output > 50) {
      
      if (center > 50) {
        velocity(255, 255);
        hard_left();
      } else {
        velocity(255, 255);
        soft_left();
      }
    }
    else {
      
      if (center > 50) {
        velocity(255, 255);
        hard_right();
      } else {
        velocity(255, 255);
        soft_right();
      }
    }
    prev_right = right;
    prev_left = left;
    prev_center = center;
}
}
