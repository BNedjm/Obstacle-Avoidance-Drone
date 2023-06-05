#include "TimerOne.h"
#include <Servo.h>

// Customized pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
#define MOD_PULSE_LENGTH 1500 // Mid pulse length in µs
#define PLS_PULSE_LENGTH = 1650
#define MNS_PULSE_LENGTH = 1350

// We create variables for the time width values of each PWM input signal
unsigned long counter_1, counter_2, counter_3, counter_4, current_count;

// We create 4 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state;

// To store the 1000us to 2000us value we create variables and store each channel
int input_YAW;      //In my case channel 4 of the receiver and pin D12 of arduino
int input_PITCH;    //In my case channel 2 of the receiver and pin D9 of arduino
int input_ROLL;     //In my case channel 1 of the receiver and pin D8 of arduino
int input_THROTTLE; //In my case channel 3 of the receiver and pin D10 of arduino

// Control signals to write the PWM signal for each channel
Servo output_YAW;      //In my case channel 4 of the receiver and pin D6 of arduino
Servo output_PITCH;    //In my case channel 2 of the receiver and pin D3 of arduino
Servo output_ROLL;     //In my case channel 1 of the receiver and pin D2 of arduino
Servo output_THROTTLE; //In my case channel 3 of the receiver and pin D5 of arduino

// Ultrasonic sensor pins
const int front_trigPin = 4;
const int front_echoPin = 7;

// Timing variables
unsigned long previousTime = 0;
const unsigned long interval = 1000; // Measurement interval in milliseconds

void setup() { 
   
  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
  PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change.                                             
  PCMSK0 |= (1 << PCINT2);  //Set pin D10 trigger an interrupt on state change.                                               
  PCMSK0 |= (1 << PCINT4);  //Set pin D12 trigger an interrupt on state change.  

  output_YAW.attach(6);       //attach pin D6 to output YAW
  output_PITCH.attach(3);     //attach pin D3 to output PITCH
  output_ROLL.attach(2);      //attach pin D2 to output ROLL
  output_THROTTLE.attach(5);  //attach pin D5 to output THROTTLE

  // Define ultrasonic sensor pins as input/output
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Serial settings
  Serial.begin(9600);  
  while (!Serial); 

}

void loop() {

  input_YAW;
  input_PITCH;
  input_ROLL;
  input_THROTTLE;

  // Write the output channels with the received values accordingly
  output_YAW.writeMicroseconds(input_YAW); 
  output_PITCH.writeMicroseconds(input_PITCH);
  output_ROLL.writeMicroseconds(input_ROLL);
  output_THROTTLE.writeMicroseconds(input_THROTTLE);

  // Call the function to get the distance measurement
  long distance = getDistance(front_trigPin, front_echoPin);

  

  // Check if it's time for a new measurement
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= interval) {
    
    // Print the distance to the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm || ");

    // Update the previous time to the current time
    previousTime = currentTime;
  }

}

// Drone arming function

void arm(){
  output_THROTTLE.writeMicroseconds(MNS_PULSE_LENGTH);
  output_YAW.writeMicroseconds(MNS_PULSE_LENGTH);
}

// Movement functions

void moveForward(){
  output_PITCH.writeMicroseconds(PLS_PULSE_LENGTH);
}

void moveBackward(){
  output_PITCH.writeMicroseconds(MNS_PULSE_LENGTH);
}

void takeOff(){
  output_THROTTLE.writeMicroseconds(PLS_PULSE_LENGTH);
}

void land(){
  output_THROTTLE.writeMicroseconds(MNS_PULSE_LENGTH);
}

// UltraSonic sensor distance function 

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

ISR(PCINT0_vect){
  
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PINB & B00000001){                              //pin D8 -- B00000001
    if(last_CH1_state == 0){
      last_CH1_state = 1;
      counter_1 = current_count;
    }
  }
  else if(last_CH1_state == 1){                
    last_CH1_state = 0;    
    input_ROLL = current_count - counter_1;
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
