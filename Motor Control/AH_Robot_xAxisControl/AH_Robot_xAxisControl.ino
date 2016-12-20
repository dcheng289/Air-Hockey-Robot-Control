// For reading serial packet
char term = 't';
 
// X-Axis: COMM4

// Pin definitions
int xDirPin = 11; // x-axis Direction pin
int xStepPin = 12;  // x-axis Step Pin
int LEDpin = 13; // LED pin (debugging)

// Movement Variables
bool robot_direction;
int t, pred_x, pred_y, cur_x, cur_y, move_x, move_y;

// Static Definitions
int x_mid = 250;     // re-center axis to the middle, halfway between upperBound and lowerBound
int maxMove_x = 1200; // 1250 steps = 125 px * 10 step/px = 113 steps/in * 11 in
int delayTime_x = 350; // min delay time to maximize motor speed
int x_Calibration = 10; // steps/px

void setup() {
  pinMode(xDirPin, OUTPUT);
  pinMode(xStepPin, OUTPUT);
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, LOW);
  Serial.begin(115200); // // opens serial port, sets data rate to 115200 bps

  pred_x = x_mid;
  move_x = 0;
}

void loop() {
  while(Serial.available() == 0) // Do nothing until serial input is received
  { }

  while(Serial.available() != 0) // loop thru the whole serial
   {
    delay(10);

    char command = Serial.read();

    // Reading Serial Packet
        if (command == 's')    // 's' detected for start flag
        {
          t = numberCollect();
          pred_x = numberCollect();
          pred_y = numberCollect();
          cur_x = numberCollect();
          cur_y = numberCollect();
          numberCollect(); // call one more time to clear serial
              
          move_x = pred_x - x_mid; // move relative to the middle
          move_motor(move_x);
        }

        if (command == 'm') // Move back to center
          move_motor(-(move_x+2));  // move 3 less pixel back
        
   }
} // end loop


void move_motor(int move_x) {
    
      digitalWrite(LEDpin, HIGH);
      
      // Pixel to Step conversion
      move_x *= x_Calibration; 

// X-Axis Motion
     // Select Direction
          if (move_x > 0)         // set direction to LOW for positive           
            digitalWrite(xDirPin, LOW);
          else                    // set direction to HIGH for negative
            digitalWrite(xDirPin, HIGH);

          move_x = abs(move_x);   // set to absolute value
          if (move_x > maxMove_x)  // maxMove_x is max # steps before we hit wall
            move_x = maxMove_x;
            
// Move X Motor
           delay(10);

            for (int i = 0; i < move_x ; i++) // X-Axis Movement
            {
              digitalWrite(xStepPin, LOW);  // This LOW to HIGH change is what creates the
              digitalWrite(xStepPin, HIGH); // "Rising Edge" so the easydriver knows to when to step.
              delayMicroseconds(delayTime_x);      // This delay time is close to top speed for this
            }

            Serial.println(move_x);
            digitalWrite(LEDpin, LOW);
}

int numberCollect(){        // Reads from serial, returns int
  int numberInput = 0;
  char buf;   

  delay(5);
  buf = Serial.read();

  if (buf == 'r')       // string ending character 'r' 
    return -1;
  
  while (true) {        
    if (buf == term)  // terminating char 't'
      return numberInput;
      
   // Loop until we hit terminating character
    numberInput *= 10;        // i.e. 151 = 100 + 50 + 1
    numberInput += (buf - '0'); // -'0' is to convert from ASCII to int
    buf = Serial.read();
  }
  return numberInput;
}


/* Test messages
 * X-Axis only        +50 px        s20t300t50t100t400tr
 * X-Axis only        -50 px        s20t200t50t100t400tr
 * X-Axis only (max)  +125 px       s20t375t50t100t400tr
 * X-Axis only (max)  -125 px       s20t125t50t100t400tr
 * 
 * Y-Axis only        +25 px        s20t250t75t104t344tr  
 * Y-Axis only (max)  +75 px        s20t250t125t104t344tr
 * 
 * Both               +50 px        s20t300t100t100t400tr
 *
 *  Serial Packet Format
    s      0       t     100     t     100     t     200    t    200    t    r
    start  time    term  pred_x  term  pred_y  term  cur_x term  cur_y term  carriagereturn
*/



