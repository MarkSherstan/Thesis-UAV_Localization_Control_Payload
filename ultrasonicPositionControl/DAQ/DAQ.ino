// Define variables
const int trigPin[] = {7, 9, 11};
const int echoPin[] = {8, 10, 12};
int sensorCount = 3;
int distance[3];
long duration;


void setup() {
  // Setup serial port
  Serial.begin(9600);

  // Setup digital pins input and output
  for(int ii=0; ii<sensorCount; ii++){
     pinMode(trigPin[ii], OUTPUT);
     pinMode(echoPin[ii], INPUT);
  }
}


void loop() {
  // Clear the trigPins
  trigPinState(0);
  delayMicroseconds(3);

  // Set the trigPin on HIGH for 10 micro seconds
  trigPinState(1);
  delayMicroseconds(10);
  trigPinState(0);

  // Read the echoPin, returns the sound wave travel time in microseconds
  // distance = traveltime x speed of sound [cm/us] x 1/2
  for(int ii=0; ii<sensorCount; ii++){
    duration = pulseIn(echoPin[ii], HIGH);
    distance[ii] = duration * 0.0343 * 0.5;
  }

  // Write bytes
  writeBytes(&distance[0], &distance[1], &distance[2]);
}


void trigPinState(bool* state){
  // Set the state of the trigger pins
  for(int ii=0; ii<sensorCount; ii++){
    digitalWrite(trigPin[ii], state);
  }
}


void writeBytes(int* data1, int* data2, int* data3){
  // Cast to a byte pointer
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);

  // Byte array with header for transmission
  byte buf[4] = {0x9F, 0x6E, byteData1[0], byteData1[1]};

  // Write the byte
  Serial.write(buf, 4);
}
