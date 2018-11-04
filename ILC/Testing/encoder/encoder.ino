#define outputA 6
#define outputB 7

int counter = 0;
int angle = 0;
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

     // Track total rotations and bound between -20 and 20
     if (counter >= 40){
       counter = 0;
     } else if (counter <= -40){
       counter = 0;
     }

     // Print angle to screen. 360 / 40 = 9
     //Serial.println(counter);
     Serial.println(counter*9);
   }
   
   // Updates the previous state of the outputA with the current state
   aLastState = aState;
}
