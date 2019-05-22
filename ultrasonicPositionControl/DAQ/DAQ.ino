// Define variables
const int pingPin[] = {20, 19, 21}; // North, East, Down
int sensorCount = 3;
int distance[3];
long duration;


void setup() {
  // Setup serial port
  Serial.begin(9600);
}


void loop() {
  // Give a short LOW pulse to make sure there is a clean HIGH pulse
  pingPinOutput();

  pingPinState(0);
  delayMicroseconds(2);

  pingPinState(1);
  delayMicroseconds(5);

  pingPinState(0);

    // Read the echoPin, returns the sound wave travel time in microseconds
    // distance = traveltime x speed of sound [cm/us] x 1/2
  pingPinInput();

  for(int ii=0; ii<sensorCount; ii++){
    duration = pulseIn(pingPin[ii], HIGH);
    distance[ii] = duration * 0.0343 * 0.5;
  }

  // Write bytes or display info to user
  //writeBytes(&distance[0], &distance[1], &distance[2]);
  Serial.print(distance[0]); Serial.print(distance[1]); Serial.println(distance[2]);
}


void pingPinOutput(){
  // Setup digital pins as output
  for(int ii=0; ii<sensorCount; ii++){
     pinMode(pingPin[ii], OUTPUT);
  }
}


void pingPinInput(){
  // Setup digital pins as input
  for(int ii=0; ii<sensorCount; ii++){
    pinMode(pingPin[ii], INPUT);
  }
}


void pingPinState(bool* state){
  // Set the state of the digital pins
  for(int ii=0; ii<sensorCount; ii++){
    digitalWrite(pingPin[ii], state);
  }
}


void writeBytes(int* data1, int* data2, int* data3){
  // Cast to a byte pointer
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);

  // Byte array with header for transmission
  byte buf[8] = {0x9F, 0x6E,
                byteData1[0], byteData1[1],
                byteData2[0], byteData2[1],
                byteData3[0], byteData3[1]};

  // Write the byte
  Serial.write(buf, 8);
}
