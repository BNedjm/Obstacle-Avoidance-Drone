#include "TimerOne.h"

// We create variables for the time width values of each PWM input signal
unsigned long counter_2, counter_3, current_count;
unsigned long counter_6, counter_5;

// We create 2 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_CH2_state, last_CH3_state;
byte last_CH6_state, last_CH5_state;

// To store the 1000us to 2000us value we create variables and store each channel
int input_PITCH;    //In my case channel 2 of the receiver and pin D8 of arduino
int input_THROTTLE; //In my case channel 3 of the receiver and pin D12 of arduino
// Feedback variables
int feedback_PITCH;
int feedback_THROTTLE;

// Define the PWM output pins
const int PWM_PITCH_PIN = 9;    // Pin for PITCH output
const int PWM_THROTTLE_PIN = 10; // Pin for THROTTLE output
const int frequency = 50; // 50 Hz
const int tx_resolution = 1024;
const int rx_resolution = calculatePeriod(frequency); // Max pulse length during one period, which equals to one period

// Ultrasonic sensor pins
const int trigPin = 2;
const int echoPin = 3;

// Timing variables
unsigned long previousTime = 0;
const unsigned long interval = 1000; // Measurement interval in milliseconds

void setup() {

  // Define PWM pins as output
  pinMode(PWM_PITCH_PIN, OUTPUT);
  pinMode(PWM_THROTTLE_PIN, OUTPUT);

  // Define ultrasonic sensor pins as input/output
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  PCICR |= (1 << PCIE0);    // Enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  // Set pin D8 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);  // Set pin D12 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT5);  // Set pin D5 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT6);  // Set pin D6 trigger an interrupt on state change.
  
  // TimerOne settings
  Timer1.initialize(calculatePeriod(frequency));         // Initialize timer1 to 50Hz
  Timer1.pwm(PWM_PITCH_PIN, 1); 
  Timer1.pwm(PWM_THROTTLE_PIN, 1); 

  // Serial settings
  Serial.begin(9600);  
  while (!Serial); 

}

void loop() {

  input_PITCH;
  input_THROTTLE;
  feedback_PITCH;
  feedback_THROTTLE;

  // Map the input values to fit within the Serial Plotter range
  int value_PITCH = map(input_PITCH, 0, rx_resolution , 0, tx_resolution - 1);
  int value_THROTTLE = map(input_THROTTLE, 0, rx_resolution, 0, tx_resolution - 1);
  // Map the feedback input values to fit within the Serial Plotter range
  int feedback_value_PITCH = map(feedback_PITCH, 0, tx_resolution - 1, 0, rx_resolution);
  int feedback_value_THROTTLE = map(feedback_THROTTLE, 0, tx_resolution - 1, 0, rx_resolution);
  
  // Write the duty cycle
  Timer1.setPwmDuty(PWM_PITCH_PIN, value_PITCH);
  Timer1.setPwmDuty(PWM_THROTTLE_PIN, value_THROTTLE);

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
    Serial.print(" MAPPED TO ");
    Serial.print(value_THROTTLE);
    Serial.print(" IN-->OUT ");
    Serial.print(feedback_THROTTLE);
    Serial.print(" Gives ");
    Serial.print(feedback_value_THROTTLE);
    Serial.print(" || ");

    Serial.print("PITCH: ");
    Serial.print(input_PITCH);
    Serial.print(" MAPPED TO ");
    Serial.print(value_PITCH);
    Serial.print(" IN-->OUT ");
    Serial.print(feedback_PITCH);
    Serial.print(" Gives ");
    Serial.println(feedback_value_PITCH);

    // Update the previous time to the current time
    previousTime = currentTime;
  }

}

// Convert frequency to period in microseconds

long calculatePeriod(int frequency) {
  return 1000000 / frequency;
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

// This is the interruption routine

ISR(PCINT0_vect){

  current_count = micros();
  
  // Channel 2 (PITCH input)
  if(PINB & B00000001) {  // Pin D8 - B00000001
    if(last_CH2_state == 0) {
      last_CH2_state = 1;
      counter_2 = current_count;
    }
  }
  else if(last_CH2_state == 1) {
    last_CH2_state = 0;
    input_PITCH = current_count - counter_2;
  }

  // Channel 3 (THROTTLE input)
  if(PINB & B00010000) {  // Pin D12 - B00010000
    if(last_CH3_state == 0) {
      last_CH3_state = 1;
      counter_3 = current_count;
    }
  }
  else if(last_CH3_state == 1) {
    last_CH3_state = 0;
    input_THROTTLE = current_count - counter_3;
  }

  // Feedback input
  // Channel 6 (feedback THROTTLE input)
  if(PIND & B01000000) {  // Pin D6 - B01000000
    if(last_CH6_state == 0) {
      last_CH6_state = 1;
      counter_6 = current_count;
    }
  }
  else if(last_CH6_state == 1) {
    last_CH6_state = 0;
    feedback_THROTTLE = current_count - counter_6;
  }

  // Channel 5 (feedback PITCH input)
  if(PIND & B00100000) {  // Pin D5 - B00100000
    if(last_CH5_state == 0) {
      last_CH5_state = 1;
      counter_5 = current_count;
    }
  }
  else if(last_CH5_state == 1) {
    last_CH5_state = 0;
    feedback_PITCH = current_count - counter_5;
  }
}
