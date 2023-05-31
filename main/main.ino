// #include <TimerOne.h>

//We create variables for the time width values of each PWM input signal
unsigned long counter_1, counter_2, counter_3, counter_4, current_count;

//We create 4 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state;

//To store the 1000us to 2000us value we create variables and store each channel
int input_YAW;      //In my case channel 4 of the receiver and pin D12 of arduino
int input_PITCH;    //In my case channel 2 of the receiver and pin D9 of arduino
int input_ROLL;     //In my case channel 1 of the receiver and pin D8 of arduino
int input_THROTTLE; //In my case channel 3 of the receiver and pin D10 of arduino

// Define the PWM output pins
const int PWM_YAW_PIN = 3;      // Pin for YAW output
const int PWM_PITCH_PIN = 5;    // Pin for PITCH output
// const int PWM_ROLL_PIN = 6;     // Pin for ROLL output
const int PWM_ROLL_PIN = 9;     // Pin for ROLL output
const int PWM_THROTTLE_PIN = 11; // Pin for THROTTLE output
const int resolution = 1024;


void setup() {
  pinMode(PWM_ROLL_PIN, OUTPUT);
  // Timer1.initialize(20000);
  // Timer1.start();

  // Timer1.pwm(PWM_ROLL_PIN, 0);
  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
  // PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change.                                             
  PCMSK0 |= (1 << PCINT2);  //Set pin D10 trigger an interrupt on state change.                                               
  PCMSK0 |= (1 << PCINT4);  //Set pin D12 trigger an interrupt on state change. 
   
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

  int value_YAW = map(input_YAW, 1000, 2003, 0, 255); // Map the input values to fit within the Serial Plotter range
  int value_PITCH = map(input_PITCH, 1000, 2003, 0, 255);
  int value_ROLL = map(input_ROLL, 1000, 2003, 0, 255);
  // int value_THROTTLE = map(input_THROTTLE, 1000, 2003, 0, 255);
  // int value_THROTTLE = map(input_THROTTLE, 0, 2003, 0, 255);
  uint16_t value_THROTTLE = map(input_THROTTLE, 0, 2000, 0, 1023);

  Serial.print(input_YAW);
  Serial.print(",");
  Serial.print(input_PITCH);
  Serial.print(",");
  Serial.print(input_ROLL);
  Serial.print(",");
  Serial.println(input_THROTTLE);

  Serial.print(input_THROTTLE);
  Serial.print(",");
  Serial.println(value_THROTTLE);
  
  analogWrite(3, value_YAW);
  analogWrite(4, value_PITCH);
  analogWrite(5, value_ROLL);
  // analogWrite(6, value_THROTTLE);
  // Timer1.setPwmDuty(PWM_ROLL_PIN, value_THROTTLE); // Set the PWM duty cycle
  analogWrite10Bit(PWM_ROLL_PIN, value_THROTTLE);

}



// Custom 10-bit PWM function
void analogWrite10Bit(int pin, int value) {
  if (value < 1000) value = 1000;
  if (value > resolution - 1) value = resolution - 1;
  uint16_t dutyCycle = map(value, 0, resolution - 1, 0, 255);
  analogWrite(pin, dutyCycle);
}

//This is the interruption routine
//----------------------------------------------

ISR(PCINT0_vect){
//First we take the current count value in micro seconds using the micros() function
  
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

