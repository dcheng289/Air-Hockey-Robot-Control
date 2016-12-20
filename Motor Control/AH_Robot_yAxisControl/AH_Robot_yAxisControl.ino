// For reading serial packet
char term = 't';

// Y-Axis: COMM5

// Pin definitions
int yDirPin = 11; // x-axis Direction pin
int yStepPin = 12;  // x-axis Step Pin
int LEDpin = 13; // LED pin (debugging)

// Movement Variables
bool robot_direction;
int t, pred_x, pred_y, cur_x, cur_y, move_x, move_y;

// Static Definitions
int maxMove_y = 900; // Maximum steps forward movement (100 px forward)      
int delayTime_y = 1000; // min delay time to maximize motor speed
int y_Calibration = 9; // steps/px
int zero_y = 50;  // Forward position is 50 pixels forward
// Pixels: 11.33 px/in

void setup() {
  pinMode(yDirPin, OUTPUT);
  pinMode(yStepPin, OUTPUT);
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, LOW);
  Serial.begin(115200); // // opens serial port, sets data rate to 115200 bps
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
              
          move_y = pred_y - zero_y;  // move y-axis motor relative to pre-defined zero point
          move_motor(move_y);
        }

        if (command == 'm') // Move back to center
          move_motor( -(move_y-1) ); // 1 less pixels = 9 less steps to compensate for imperfect int conversion to steps
        
   }
} // end loop


void move_motor(int move_y) {
    
      digitalWrite(LEDpin, HIGH);
      
      // Pixel to Step conversion
      move_y *= y_Calibration; 

// Y-Axis Motion
     // Select Direction
          if (move_y > 0)         // set direction to LOW for positive           
            digitalWrite(yDirPin, LOW);
          else                    // set direction to HIGH for negative
            digitalWrite(yDirPin, HIGH);

          move_y = abs(move_y);   // set to absolute value
          if (move_y > maxMove_y)  // maxMove_y is max # steps before we hit wall
            move_y = maxMove_y;
            
// Move Y Motor
           delay(10);

            for (int i = 0; i < move_y ; i++) // Y-Axis Movement
            {
              digitalWrite(yStepPin, LOW);  // This LOW to HIGH change is what creates the
              digitalWrite(yStepPin, HIGH); // "Rising Edge" so the easydriver knows to when to step.
              delayMicroseconds(delayTime_y);      // This delay time is close to top speed for this
            }

            Serial.println(move_y);
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


/* Test Messages
 * X-Axis only        +50 px        s20t300t50t100t400tr
 * X-Axis only        -50 px        s20t200t50t100t400tr
 * X-Axis only (max)  +125 px       s20t375t50t100t400tr
 * X-Axis only (max)  -125 px       s20t125t50t100t400tr
 * 
 * Y-Axis only        +25 px        s20t250t75t104t344tr  
 * Y-Axis only (max)  +50 px        s20t250t100t104t344tr
 * 
 * // Max Movement: 125 px
 * 
 * Both               +50 px        s20t292t100t100t400tr
 *
 *  Serial Packet Format
    s      0       t     100     t     100     t     200    t    200    t    r
    start  time    term  pred_x  term  pred_y  term  cur_x term  cur_y term  carriagereturn
*/


