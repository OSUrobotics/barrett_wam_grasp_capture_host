/*
prev and curr hold current and previous averages of readings
from potentiometers. 0-2 are read from the sliders, and 3 is
from the trimpot
*/
float prev0 = 0;
float curr0 = 0;

float prev1 = 0;
float curr1 = 0;

float prev2 = 0;
float curr2 = 0;

float prev3 = 0;
float curr3 = 0;

/*
readings holds an array of 50 readings to smooth the input from
the potentiometers
*/
float readings0[50];
float readings1[50];
float readings2[50];
float readings3[50];

float readings0_length = sizeof(readings0)/sizeof(*readings0);
float readings1_length = sizeof(readings1)/sizeof(*readings1);
float readings2_length = sizeof(readings2)/sizeof(*readings2);
float readings3_length = sizeof(readings3)/sizeof(*readings3);

/*
int i is used to increment through each readings array

finger_scale is used to map the analog readings from the Arduino's
0-1023 scale to the Barrett's finger scale of 0-2.5 radians

spread scale is used to map the analog scale to a the barret 
spread scale of 0-3.14 radians
*/
int i = 0;

float finger_scale = 2.5/1023;
float spread_scale = 3.14/1023;

/*
buffer holds the string that the Arduino will pass to the python code running
the hand

avg0-3 are the strings that hold the average from each potentiometer that will
be written to the buffer string
*/

char buffer[25];

char avg0[5];
char avg1[5];
char avg2[5];
char avg3[5];

/*
intialize Serial communication with 9600 baud rate
initialize functions to get average and look for change in averages
*/
void setup(){
  Serial.begin(9600);
  float average(float* array, int length);
  boolean change();
}



void loop(){
  //store analog values to readings arrays
  readings0[i] = analogRead(A0);
  readings1[i] = analogRead(A1);
  readings2[i] = analogRead(A2);
  readings3[i] = analogRead(A3);
  i++;

  //when array is full, take average
  if(i == 50){
    i = 0;
    curr0 = average(readings0, readings0_length);
    curr1 = average(readings1, readings1_length);
    curr2 = average(readings2, readings2_length);
    curr3 = average(readings3, readings3_length);
    
    //look for significant changes in averages
    if(change()){
      //if change occurs, write new averages to string and write to Serial
      dtostrf(curr0*finger_scale, 4, 2, avg0);
      dtostrf(curr1*finger_scale, 4, 2, avg1);
      dtostrf(curr2*finger_scale, 4, 2, avg2);
      dtostrf(curr3*spread_scale, 4, 2, avg3);
      sprintf(buffer, "%s, %s, %s, %s", avg0, avg1, avg2, avg3);
      
      Serial.println(buffer);
      prev0 = curr0;
      prev1 = curr1;
      prev2 = curr2;
      prev3 = curr3;
    }
  }
  
  delay(10);
}

//takes in array of floats and length of array, solves for average
float average(float array[], int length){
  int k;
  float sum = 0;
  for(k = 0; k < length; k++){
    sum += array[k];
  }
  return sum/length;
}

//returns true if any of the current averages changes by 3
boolean change(){
  if(curr0 < prev0 - 3 || curr0 > prev0 + 3 || curr1 < prev1 - 3 || curr1 > prev1 + 3 || curr2 < prev2 - 3 || curr2 > prev2 + 3 || curr3 < prev3 - 3 || curr3 > prev3 + 3){
    return true;
  }else{
    return false;
  }
}
