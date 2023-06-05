#include "TimerOne.h"
#include <Servo.h>

// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 
#define MOD_PULSE_LENGTH 1500 // Maximum pulse length in µs

//We create variables for the time width values of each PWM input signal
unsigned long counter_1, counter_2, counter_3, counter_4, current_count;

//We create 4 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state;

//To store the 1000us to 2000us value we create variables and store each channel
int input_YAW;      //In my case channel 4 of the receiver and pin D12 of arduino
int input_PITCH;    //In my case channel 2 of the receiver and pin D9 of arduino
int input_ROLL;     //In my case channel 1 of the receiver and pin D8 of arduino
int input_THROTTLE; //In my case channel 3 of the receiver and pin D10 of arduino

// Control signals
Servo output_YAW;      //In my case channel 4 of the receiver and pin D6 of arduino
Servo output_PITCH;    //In my case channel 2 of the receiver and pin D3 of arduino
Servo output_ROLL;     //In my case channel 1 of the receiver and pin D2 of arduino
Servo output_THROTTLE; //In my case channel 3 of the receiver and pin D5 of arduino

// Ultrasonic sensor pins
const int trigPin = 4;
const int echoPin = 7;

// Timing variables
unsigned long previousTime = 0;
const unsigned long interval = 1000; // Measurement interval in milliseconds

void setup() {

  /*
   * Port registers allow for lower-level and faster manipulation of the i/o pins of the microcontroller on an Arduino board. 
   * The chips used on the Arduino board (the ATmega8 and ATmega168) have three ports:
     -B (digital pin 8 to 13)
     -C (analog input pins)
     -D (digital pins 0 to 7)
   
  //All Arduino (Atmega) digital pins are inputs when you begin...
  */  
   
  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
  PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change.                                             
  PCMSK0 |= (1 << PCINT2);  //Set pin D10 trigger an interrupt on state change.                                               
  PCMSK0 |= (1 << PCINT4);  //Set pin D12 trigger an interrupt on state change.  

  output_YAW.attach(6); 
  output_PITCH.attach(3);
  output_ROLL.attach(2);
  output_THROTTLE.attach(5);

  // Define ultrasonic sensor pins as input/output
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Serial settings
  Serial.begin(9600);  
  while (!Serial); 

}

void loop() {

  /*
   * Ok, so in the loop the only thing that we do in this example is to print
   * the received values on the Serial monitor. The PWM values are read in the ISR below.
   */ 

  input_YAW;
  input_PITCH;
  input_ROLL;
  input_THROTTLE;

  // Update the plots with the received values
  output_YAW.writeMicroseconds(input_YAW); 
  output_PITCH.writeMicroseconds(input_PITCH);
  output_ROLL.writeMicroseconds(input_ROLL);
  output_THROTTLE.writeMicroseconds(input_THROTTLE);

  // Check if it's time for a new measurement
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= interval) {
    // Call the function to get the distance measurement
    long distance = getDistance(trigPin, echoPin);

    // Print the distance to the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm || ");

    // More Serial coms
    Serial.print("THROTTLE: ");
    Serial.print(input_THROTTLE);
    Serial.print(" || ");

    Serial.print("PITCH: ");
    Serial.print(input_PITCH);
    Serial.println(" || ");

    // Update the previous time to the current time
    previousTime = currentTime;
  }

}


// UltraSonic distance function 

long getDistance(int trigPin, int echoPin) {
  // Send a short pulse to trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  long distance = duration * 0.034 / 2;

  return distance;
}

//This is the interruption routine
//----------------------------------------------

ISR(PCINT0_vect){
  
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PINB & B00000001){                              //We make an AND with the pin state register, We verify if pin 8 is HIGH???
    if(last_CH1_state == 0){                         //If the last state was 0, then we have a state change...
      last_CH1_state = 1;                            //Store the current state into the last state for the next loop
      counter_1 = current_count;                     //Set counter_1 to current value.
    }
  }
  else if(last_CH1_state == 1){                      //If pin 8 is LOW and the last state was HIGH then we have a state change      
    last_CH1_state = 0;                              //Store the current state into the last state for the next loop
    input_ROLL = current_count - counter_1;   //We make the time difference. Channel 1 is current_time - timer_1.
  }

  ///////////////////////////////////////Channel 2
  if(PINB & B00000010 ){                             //pin D9 -- B00000010                                              
    if(last_CH2_state == 0){                                               
      last_CH2_state = 1;                                                   
      counter_2 = current_count;                                             
    }
  }
  else if(last_CH2_state == 1){                                           
    last_CH2_state = 0;                                                     
    input_PITCH = current_count - counter_2;                             
  }

  ///////////////////////////////////////Channel 3
  if(PINB & B00000100 ){                             //pin D10 - B00000100                                         
    if(last_CH3_state == 0){                                             
      last_CH3_state = 1;                                                  
      counter_3 = current_count;                                               
    }
  }
  else if(last_CH3_state == 1){                                             
    last_CH3_state = 0;                                                    
    input_THROTTLE = current_count - counter_3;                            

  }
  
  ///////////////////////////////////////Channel 4
  if(PINB & B00010000 ){                             //pin D12  -- B00010000                      
    if(last_CH4_state == 0){                                               
      last_CH4_state = 1;                                                   
      counter_4 = current_count;                                              
    }
  }
  else if(last_CH4_state == 1){                                             
    last_CH4_state = 0;                                                  
    input_YAW = current_count - counter_4;                            
  }
 
}
