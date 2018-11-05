// From bildr article: http://bildr.org/2012/08/rotary-encoder-arduino/
// https://www.arduino.cc/en/Reference/AttachInterrupt?setlang=it

// Declare some variables
int encoderPin1 = 2;
int encoderPin2 = 3;
int lastMSB = 0; // most significant bit
int lastLSB = 0; // least significant bit
long lastencoderValue = 0;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;


void setup() {
  Serial.begin (9600);

  // declare pin 2 and 3 as input, turn pullup resistor on and call updateEncoder()
  // when there is a change
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  digitalWrite(encoderPin1, HIGH);
  digitalWrite(encoderPin2, HIGH);
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
}


void loop(){
  Serial.println(encoderValue);
}


void updateEncoder(){
  // Get data
  int MSB = digitalRead(encoderPin1);
  int LSB = digitalRead(encoderPin2);

  // Convert the 2 pin values to single number using bit shift and add to previous value
  int encoded = (MSB << 1) |LSB;
  int sum = (lastEncoded << 2) | encoded;

  // Calculate if there should be an increase or decrease in encoder value then store
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
  lastEncoded = encoded;
}
