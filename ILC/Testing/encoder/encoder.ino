#define outputA 6
#define outputB 7

int counter = 0;
int aState;
int aLastState;


void setup() {
  Serial.begin(9600);

  pinMode(outputA,INPUT);
  pinMode(outputB,INPUT);

  // Reads the initial state of the outputA
  aLastState = digitalRead(outputA);
}


void loop() {
   aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured

   if (aState != aLastState){
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) {
       counter ++;
     } else {
       counter --;
     }
     Serial.print("Position: ");
     Serial.println(counter);
   }
   // Updates the previous state of the outputA with the current state
   aLastState = aState;
}
