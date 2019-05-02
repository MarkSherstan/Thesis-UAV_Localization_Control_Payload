// Define variables
const int trigPin = 9;
const int echoPin = 10;
long duration;
int distance;


void setup() {
  // Setup serial port
  Serial.begin(9600);

  // Setup digital pins input and output
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}


void loop() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  // Set the trigPin on HIGH for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // distance = traveltime x speed of sound [cm/us] x 1/2
  distance = duration * 0.0343 * 0.5;

  // Write bytes
  writeBytes(&distance);
}


void writeBytes(int* data1){
  // Cast to a byte pointer
  byte* byteData1 = (byte*)(data1);

  // Byte array with header for transmission
  byte buf[4] = {0x9F, 0x6E, byteData1[0], byteData1[1]};

  // Write the byte
  Serial.write(buf, 4);
}
