// Define variables
const int pingPinArray[] = {20, 19, 21}; // North, East, Down
int sensorCount = 3;
int distance[3];
long duration;


void setup() {
  // Setup serial port
  Serial.begin(9600);
}


void loop() {
  // Loop through all the ping pins and get a distance
  for(int ii=0; ii<sensorCount; ii++){
     distance[ii] = getDistance(pingPinArray[ii]);
  }

  // Write bytes or display info to user
  //writeBytes(&distance[0], &distance[1], &distance[2]);
  Serial.print(distance[0]); Serial.print(" , "); Serial.print(distance[1]); Serial.print(" , "); Serial.println(distance[2]);
}


int getDistance(int* pingPin){
  // Give a short LOW pulse to make sure there is a clean HIGH pulse for trigger
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // Read the echo
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // distance = traveltime x speed of sound [cm/us] x 1/2
  return duration * 0.0343 * 0.5;
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
