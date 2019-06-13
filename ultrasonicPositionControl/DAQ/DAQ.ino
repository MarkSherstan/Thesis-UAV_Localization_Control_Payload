// Define variables
const int pingPinArray[] = {11, 9, 10, 5, 6}; // North1, North2, East1, East2, Down
int sensorCount = 5;
int distance[5];
long duration;


void setup() {
  // Setup serial port
  Serial.begin(9600);
}


void loop() {
  // Loop through all the ping pins and get distances (North, East, Down)
  for(int ii=0; ii<sensorCount; ii++){
     distance[ii] = getDistance(pingPinArray[ii]);
  }

  // Write bytes or display info to user
  writeBytes(&distance[0], &distance[1], &distance[2], &distance[3], &distance[4]);

  // Serial.print(distance[0]); Serial.print("\t");
  // Serial.print(distance[1]); Serial.print("\t");
  // Serial.print(distance[2]); Serial.print("\t");
  // Serial.print(distance[3]); Serial.print("\t");
  // Serial.println(distance[4]);
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


void writeBytes(int* data0, int* data1, int* data2, int* data3, int* data4){
  // Cast to a byte pointer
  byte* byteData0 = (byte*)(data0);
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);

  // Byte array with header for transmission
  byte buf[12] = {0x9F, 0x6E,
                  byteData0[0], byteData0[1],
                  byteData1[0], byteData1[1],
                  byteData2[0], byteData2[1],
                  byteData3[0], byteData3[1],
                  byteData4[0], byteData4[1]};

  // Write the byte
  Serial.write(buf, 12);
}
