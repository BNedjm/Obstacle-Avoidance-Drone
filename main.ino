
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
const int PWM_ROLL_PIN = 6;     // Pin for ROLL output
const int PWM_THROTTLE_PIN = 11; // Pin for THROTTLE output

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
                                                 
                                               
  
  
  //Start the serial in order to see the result on the monitor
  //Remember to select the same baud rate on the serial monitor
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
  int value_YAW = map(input_YAW, 1001, 2003, 0, 1023); // Map the input values to fit within the Serial Plotter range
  int value_PITCH = map(input_PITCH, 1001, 2002, 0, 1023);
  int value_ROLL = map(input_ROLL, 1001, 2003, 0, 1023);
  int value_THROTTLE = map(input_THROTTLE, 1000, 2002, 0, 1023);

  // Output the mapped values to PWM pins
  analogWrite(PWM_YAW_PIN, input_YAW); //is good
  analogWrite(PWM_PITCH_PIN, input_ROLL); // sync input_ROLL
  analogWrite(PWM_ROLL_PIN, input_THROTTLE); // sync
  analogWrite(PWM_THROTTLE_PIN, input_PITCH); // sync input_PITCH

  // Send the values as comma-separated values to the Serial Monitor
  // Serial.print(value_YAW);
  // Serial.print(",");
  // Serial.print(value_PITCH);
  // Serial.print(",");
  // Serial.print(value_ROLL);
  // Serial.print(",");
  Serial.println(input_THROTTLE);
  
  // // Update the plot with the received value
  // int value_YAW = map(input_YAW, 1001, 2003, 0, 1023); // Map the input value to fit within the Serial Plotter range

  // // Send the value to the Serial Monitor
  // Serial.println(value_YAW);

  // Wait for a short delay
  delay(500);
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

