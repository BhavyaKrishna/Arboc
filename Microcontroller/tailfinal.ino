#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 2
#include<avr/io.h>
#include<avr/interrupt.h>
int K=0;

int main()
{sei();
  DDRB|=(1<<PINB5);
  TCCR1B|=(1<<CS11)|(1<<WGM12);
  TIMSK1|=(1<<OCIE1A);
  OCR1A=10000;
   DDRD|=0XF0;
   PORTD|=(1<<4);
  PORTD&=~(1<<5);
  Serial.begin(9600);
  int frequency = 0;
  int red=0,blue=0,green=0;
  
while(1)
{
   PORTD&=~(1<<6);
   PORTD&=~(1<<7);
  
  // Reading the output frequency
  red = pulseIn1(sensorOut, LOW,1000);
  // Printing the value on the serial monitor
  Serial.print("R= ");//printing name
  Serial.print(red);//printing RED color frequency
  Serial.print("  ");
  //Serial.print(clockCyclesPerMicrosecond());
  
  // Setting Green filtered photodiodes to be read
  //digitalWrite(S2,HIGH);
  //digitalWrite(S3,HIGH);
  // Reading the output frequency
   PORTD|=(1<<6);
    PORTD|=(1<<7);
  green = pulseIn1(sensorOut, LOW,1000);
  // Printing the value on the serial monitor
  Serial.print("G= ");//printing name
  Serial.print(green);//printing green color frequency
  Serial.print("  ");
  
  // Setting Blue filtered photodiodes to be read
 // digitalWrite(S2,LOW);
  //digitalWrite(S3,HIGH);
   PORTD&=~(1<<6);
   PORTD|=(1<<7);
  // Reading the output frequency
  blue = pulseIn1(sensorOut, LOW,1000);
  // Printing the value on the serial monitor
  Serial.print("B= ");//printing name
  Serial.print(blue);//printing blue color frequency
  Serial.println("  ");
 
  
  }
}
ISR(TIMER1_COMPA_vect)
{
  K++;
  if(K==40)
  {
    PORTB^=(1<<PINB5);
    K=0;
  }
}

int frequency = 0;
/*void setup() {
  DDRD|=0XF0;
  //pinMode(S1, OUTPUT);
  //pinMode(S2, OUTPUT);
 // pinMode(S3, OUTPUT);
 // pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  //digitalWrite(S0,HIGH);
  //digitalWrite(S1,LOW);
  PORTD|=(1<<4);
  PORTD&=~(1<<5);
  
  
  Serial.begin(9600);
}*/
unsigned long pulseIn1(uint8_t pin, uint8_t state, unsigned long timeout)
{
  // cache the port and bit of the pin in order to speed up the
  // pulse width measuring loop and achieve finer resolution.  calling
 // digitalRead() ;instead yields much coarser resolution.
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
                                                                                                                                                                                                                                                                                                                                                             
  uint8_t stateMask = (state ? bit : 0);
  unsigned long width = 0; // keep initialization out of time critical area
  
  // convert the timeout from microseconds to a number of times through
  // the initial loop; it takes 16 clock cycles per iteration.
  unsigned long numloops = 0;
  //unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;
  
  long maxloops =timeout;
  // wait for any previous pulse to end
  while ((*portInputRegister(port) & bit) == stateMask)
    if (numloops++ == maxloops)
      return 0;
  
  // wait for the pulse to start
  while ((*portInputRegister(port) & bit) != stateMask)
    if (numloops++ == maxloops)
      return 0;
  
  // wait for the pulse to stop
  while ((*portInputRegister(port) & bit) == stateMask) {
    if (numloops++ == maxloops)
      return 0;
    width++;
  }

  // convert the reading to microseconds. The loop has been determined
  // to be 20 clock cycles long and have about 16 clocks between the edge
  // and the start of the loop. There will be some error introduced by
  // the interrupt handlers.
  //return clockCyclesToMicroseconds(width * 21 + 16); 
 return (width * 21 + 16)/16;
}


