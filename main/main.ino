
//We create variables for the time width values of each PWM input signal
unsigned long counter_2, counter_3, current_count;

//We create 2 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_CH2_state, last_CH3_state;

//To store the 1000us to 2000us value we create variables and store each channel
int input_PITCH;    //In my case channel 2 of the receiver and pin D8 of arduino
int input_THROTTLE; //In my case channel 3 of the receiver and pin D12 of arduino

// Define the PWM output pins
const int PWM_PITCH_PIN = 9;    // Pin for PITCH output
const int PWM_THROTTLE_PIN = 10; // Pin for THROTTLE output
const int resolution = 1024;


void setup() {

  pinMode(PWM_PITCH_PIN, OUTPUT);
  pinMode(PWM_THROTTLE_PIN, OUTPUT);
  
  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change.                                               
  PCMSK0 |= (1 << PCINT4);  //Set pin D12 trigger an interrupt on state change. 
   
  // Serial.begin(9600);  
  // while (!Serial); 
}

void loop() {

  input_PITCH;
  input_THROTTLE;

  // Map the input values to fit within the Serial Plotter range
  int value_PITCH = map(input_PITCH, 0, 2000, 0, resolution - 1);
  int value_THROTTLE = map(input_THROTTLE, 0, 2000, 0, resolution - 1);


  // Serial.print(input_PITCH);
  // Serial.print(",");
  // Serial.println(input_THROTTLE);
  
  analogWrite10Bit(PWM_PITCH_PIN, value_PITCH);
  analogWrite10Bit(PWM_THROTTLE_PIN, value_THROTTLE);

}



// Custom 10-bit PWM function

void analogWrite10Bit(int pin, int value) {
  if (value < 0) value = 0;
  if (value > resolution - 1) value = resolution - 1;
  uint16_t dutyCycle = map(value, 0, resolution - 1, 0, 255);
  analogWrite(pin, dutyCycle);
}

//This is the interruption routine
//----------------------------------------------

ISR(PCINT0_vect){

  current_count = micros();
  ///////////////////////////////////////Channel 2
  if(PINB & B00000001){                                //pin D8  -- B00000001
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
  if(PINB & B00010000 ){   
    if(last_CH3_state == 0){                            //pin D12  -- B00010000    //   if(last_CH3_state == 0){                                             
      last_CH3_state = 1;                                                  
      counter_3 = current_count;                                               
    }
  }
  else if(last_CH3_state == 1){                                             
    last_CH3_state = 0;                                                    
    input_THROTTLE = current_count - counter_3;                            

  }
}