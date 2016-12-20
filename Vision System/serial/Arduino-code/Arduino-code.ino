char incomingByte = 0;   // for incoming serial data
int led = 13;
void setup()
{
  Serial.begin(9600); // // opens serial port, sets data rate to 9600 bps
 
  pinMode(led, OUTPUT);
}
 
void loop()
{ digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
 
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    for (int i = 0; i < incomingByte; i++) {
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(100);
      digitalWrite(led, LOW); 
      delay(100);
    }
    
  }
}

/*
 * 
 * int led = 13;
void setup()
{
  Serial.begin(9600); // // opens serial port, sets data rate to 9600 bps
 
  pinMode(led, OUTPUT);
}
 
void loop()
{ 
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW



  char buf;
  if (Serial.available() > 0) { // serial is available, bytes incoming

    digitalWrite(led, HIGH);
    while (buf != 13) {
    // read the incoming byte:
    buf = Serial.read();
    if (buf!= 13)
      Serial.print(buf);
    else
      Serial.println();

    }
  }
}

/*
char buf 
  while (buf != 13) {         // while last character was not a carriage return
    buf = Serial.read();
    if (buf != 13) {
    numberInput *= 10;
    numberInput += (buf - '0');
    }

    // do I need to have an exception when the value entered is greater than 8191
  }
  */

