//#include <Servo.h>

#define IR_BIT_LENGTH 12    // number of bits sent by IR remote
#define BIT_1 1000          // Binary 1 threshold (Microseconds)
#define BIT_0 400           // Binary 0 threshold (Microseconds)
#define BIT_START 2000      // Start bit threshold (Microseconds)

#define IR_PIN 2            // Sensor pin 1 wired through a 220 ohm resistor

#define DEBUG 1   

int runtime_debug = 1;      // flag to output raw IR pulse data
int output_key = 1;         // flag to print decoded key integers

int motorDifference = 30;

volatile boolean isWanderMode = false;

volatile boolean isKeyActionMode = false;

volatile boolean isFollowLeft = false;

//Servo neckServo;

int sensorPin0 = 3; 

int leftMotor_1 = 10;
int leftMotor_2 = 3;  
int leftMotor_en = 5; 

int rightMotor_1 = 7;
int rightMotor_2 = 11; 
int rightMotor_en = 6;  

int servoRotateTime = 500;
int turnTime = 600;

// ping // 0 - 9
int dangerDistance = 17;

// IR 10-24
int dangerDistanceOpposite = 51;

int straightAhead = 78;

const int pingPin = 13;

boolean isWallFollowingMode = false;

void setup() {   
  Serial.begin(9600);
  Serial.print("Ready");
  
 // neckServo.attach(13);
  //turnHead(straightAhead);
  pinMode(leftMotor_1, OUTPUT);     
  pinMode(leftMotor_2, OUTPUT);  
  pinMode(rightMotor_1, OUTPUT);     
  pinMode(rightMotor_2, OUTPUT); 

  pinMode(leftMotor_en, OUTPUT); 
  pinMode(rightMotor_en, OUTPUT); 
  
  attachInterrupt(0, testInterrupt, CHANGE);
}

void testInterrupt() {
  isWanderMode = false;
  isFollowLeft = false;

  isKeyActionMode = true;
}  

void loop() { 

  if(isKeyActionMode) {
    stop();  
    int key = get_ir_key();  
    do_response(key);
    isKeyActionMode = false;
  }
  
  if(isWanderMode) {
    wander();
  }

  if(isFollowLeft) {
    followWall(true);
  }

}

void doKeyAction() {
  int key = get_ir_key();  
  do_response(key);
}


void changeDirection() {
  if (DEBUG || runtime_debug) {Serial.print("in change direction");}
  stop();
  
  // Look left
  turnHead(0);
  delay(1000);
  int left = read_gp2d12_range();
  
  // Look right
  turnHead(150);
  delay(1000);
  int right = read_gp2d12_range();
  
  delay(1000);
  turnHead(straightAhead);
  
  // Neither are good options, turn around
  if(left < dangerDistance &&  right < dangerDistance){
    turnRight();
    delay(turnTime * 2);
    moveForward();
  }
  // Right is better
  else if(left < right){
    turnLeft();
    delay(turnTime);
    moveForward();
  }
  // Left is better
  else { 
    turnLeft();
    delay(turnTime);
    moveForward();
  }
}

void moveBackward() {
  if (DEBUG || runtime_debug) {Serial.print("in move backward");}
  analogWrite(leftMotor_en, 200);
  analogWrite(rightMotor_en, 250); 
  //turnHead(100);
  digitalWrite(leftMotor_1, HIGH);
  digitalWrite(leftMotor_2, LOW); 
  digitalWrite(rightMotor_1, LOW);
  digitalWrite(rightMotor_2, HIGH);    
}

void moveForward() {
  if (DEBUG || runtime_debug) {Serial.print("in move forward");}
  analogWrite(leftMotor_en,200);
 analogWrite(rightMotor_en, 230);   
  //turnHead(100);
  digitalWrite(leftMotor_1, LOW);
  digitalWrite(leftMotor_2, HIGH); 
  digitalWrite(rightMotor_1, HIGH);
  digitalWrite(rightMotor_2, LOW);  
}


void stop() {   
  if (DEBUG || runtime_debug) {Serial.print("in stop");}
  turnHead(straightAhead);
  analogWrite(leftMotor_en, 255);
  analogWrite(rightMotor_en, 255); 

  digitalWrite(leftMotor_1, HIGH);
  digitalWrite(leftMotor_2, HIGH); 
  digitalWrite(rightMotor_1, HIGH);
  digitalWrite(rightMotor_2, HIGH);  
}

void turnLeft() {  
  if (DEBUG || runtime_debug) {Serial.print("in turn left");}
  analogWrite(leftMotor_en, 220);
  analogWrite(rightMotor_en, 220); 
  
  digitalWrite(leftMotor_1, LOW);
  digitalWrite(leftMotor_2, HIGH); 
  digitalWrite(rightMotor_1, LOW);
  digitalWrite(rightMotor_2, HIGH);   
}

void turnRight() {  
  if (DEBUG || runtime_debug) {Serial.print("in turn right");  }
  analogWrite(leftMotor_en, 220);
  analogWrite(rightMotor_en, 220); 
  
  digitalWrite(leftMotor_1, HIGH);
  digitalWrite(leftMotor_2, LOW); 
  digitalWrite(rightMotor_1, HIGH);
  digitalWrite(rightMotor_2, LOW);     
}

void myRefresh(int delayTime){
  for(int i=0; i < delayTime/20; i++){ //delay is the total ms delay we want, 20 is the delay per iteration of the loop 
     // Servo::refresh(); 
      delay(20);
  }
}

void turnHead(int degree){
  //Serial.print("servo position 0\n");
  //neckServo.write(degree);
  myRefresh(650);
}

float read_gp2d12_range() {
  if (DEBUG || runtime_debug) {Serial.print("in read gp2d12 range");}
  byte pin = sensorPin0;
  int tmp;

  tmp = analogRead(pin);
  if (tmp < 3) {
    return -1; // invalid value
  }
  return (6787.0 /((float)tmp - 3.0)) - 4.0;
} 





long ping()
{
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  //cm = microsecondsToCentimeters(duration);
  
  if (DEBUG || runtime_debug) {Serial.print(inches);}
  if (DEBUG || runtime_debug) {Serial.print("*********** in, ");}
 // Serial.print(cm);
 // Serial.print("cm");
  if (DEBUG || runtime_debug) {Serial.println();}
   delay(100);
  return inches;
}

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}


/*
  wait for a keypress from the IR remote, and return the
  integer mapping of that key (e.g. power button on remote returns 
  the integer 1429)
*/

int get_ir_key() {
  int pulse[IR_BIT_LENGTH];
  int bits[IR_BIT_LENGTH];   
 do {} //Wait for a start bit
 while(pulseIn(IR_PIN, LOW) < BIT_START);
  read_pulse(pulse, IR_BIT_LENGTH);
  pulse_to_bits(pulse, bits, IR_BIT_LENGTH);
  return bits_to_int(bits, IR_BIT_LENGTH);

}

/* 
  respond to specific remote-control keys with different behaviors
*/

void do_response(int key) {  
  switch (key) {
    case 1437:  // record button
      if (DEBUG || runtime_debug) {Serial.println("toggle debug pulse");}
      //runtime_debug = 1 - runtime_debug;
      break;
    case 1498:  // display button
      if (DEBUG || runtime_debug) {Serial.println("Toggle key output");}
      output_key = 1 - output_key;
      break;
    case 1429:  // power button
      if (DEBUG || runtime_debug) {Serial.println("Power");}
      break;
    case 1424:  // channel up button
      if (DEBUG || runtime_debug) {Serial.println("Channel Up");}
      break;      
    case 1425:  // channel down button
      if (DEBUG || runtime_debug) {Serial.println("Channel Down");}
      break;
    case 142:  // up rocker/pause
      break;
    case 143:  // down rocker/stop
      break;      
    case 131:  // play button
      moveBackward();
      isFollowLeft = false;
      isWanderMode = false;
      break;   
    case 132:  // play button
      stop();
      isWanderMode = false;
      isFollowLeft = false;
      break;   
    case 133:  // play button    
      moveForward();
      isWanderMode = false;
      isFollowLeft = false;
      break;  
    case 135:  // play button
      turnRight();
      //easeRight(); 
      isWanderMode = false;
      isFollowLeft = false;
      break;  
    case 129:  // play button
      turnLeft();
      //easeLeft();
      isWanderMode = false;
      isFollowLeft = false;
      break;  
    case 144:  // channel +
      isWanderMode = true;
      isFollowLeft = false;
      break;      
    case 145:  // right rocker/fast forward
      isFollowLeft = true;  
      break;
    case 3352:  // play button
      isFollowLeft = false;
      break;        
    default:
      if (output_key) {
        if (DEBUG || runtime_debug) {Serial.print("Key ");}
        if (DEBUG || runtime_debug) {Serial.print(key);}
        if (DEBUG || runtime_debug) {Serial.println(" not programmed");}
        if(key != 0) {
          isFollowLeft = false;
          stop();
        }
      }
      //isFollowLeft = false; 
      break;
  }
}

/*
  use pulseIn to receive IR pulses from the remote.
  Record the length of these pulses (in ms) in an array
*/

void read_pulse(int pulse[], int num_bits) {
  for (int i = 0; i < num_bits; i++) {
    pulse[i] = pulseIn(IR_PIN, LOW);
  }
}

/*
  IR pulses encode binary "0" as a short pulse, and binary "1"
  as a long pulse.  Given an array containing pulse lengths,
  convert this to an array containing binary values
*/
void pulse_to_bits(int pulse[], int bits[], int num_bits) {
  for(int i = 0; i < num_bits ; i++) {
    if (DEBUG || runtime_debug) {Serial.println("starting"); }
    if (DEBUG || runtime_debug) { Serial.println(pulse[i]); }
 
    if(pulse[i] > BIT_1) {
      //is it a 1? 
      bits[i] = 1;
    } else if(pulse[i] > BIT_0) {
      //is it a 0? 
      bits[i] = 0;
    } else {
       //data is invalid...
      if (DEBUG || runtime_debug) {Serial.println("Error");}
    }
  }
}

/*
  convert an array of binary values to a single base-10 integer
*/
int bits_to_int(int bits[], int num_bits) {
  int result = 0;
  int seed = 1;
  
  //Convert bits to integer
  for(int i = 0 ; i < num_bits ; i++) {		  
    if(bits[i] == 1) {
	result += seed;
    }
    
    seed *= 2;
  }
  if (DEBUG || runtime_debug) {Serial.println("starting other"); }
  if (DEBUG || runtime_debug) {Serial.println(result);}
  return result;
}



void wander() {
  if (DEBUG || runtime_debug) {Serial.println("reading distance------------------");}
  int distance = read_gp2d12_range();
  if (DEBUG || runtime_debug) {Serial.println(distance);}
  if (DEBUG || runtime_debug) {Serial.println("reading distance------------------");}
  if(distance >  dangerDistance) {
    if (DEBUG || runtime_debug) {Serial.println("move forward");}
    moveForward();
    delay(50);
  } else {
    if (DEBUG || runtime_debug) {Serial.println("change direction");}
    changeDirection();
    //Serial.print("move backward");
    moveBackward();
    delay(50);
    //delay(50);
  }     
  moveForward(); 
}


void followWall(boolean isLeft) {
  int strongCounter = 0;
  int distance;
  int oppositeDistance;

  if(!isWallFollowingMode) {
    distance = ping();
    oppositeDistance = read_gp2d12_range();
   // while((distance > (dangerDistance)) && (oppositeDistance > (dangerDistanceOpposite))) {
     
    // move forward to next obstacle 
    while((distance > (dangerDistance + 5))) {
      moveForward(); 
       //delay(1000);
      distance = ping();
      Serial.println(distance);
    }

    // if obstacle near side or front, keep turning away
    if(isLeft) {
      oppositeDistance = read_gp2d12_range();
      distance = ping();
      
int dangerDistance2 = 17;
int dangerDistanceOpposite2 = 35;
      while(oppositeDistance < dangerDistanceOpposite2 || distance < dangerDistance2) {
        turnRight();
        // need this to be able to interrupt??
        delay(100);
        distance = ping();
        oppositeDistance = read_gp2d12_range();
      }
    } else {
      //turnLeft();
      //delay(turnTime);
    }
    isWallFollowingMode = true;
  }


// if wall following and there's and obstacle right in front
  distance = ping();
  while(distance < dangerDistance) {
    turnRight();
    // need this to be able to interrupt??
    delay(100);
    distance = ping();
  }  
  
  distance = ping();
  oppositeDistance = read_gp2d12_range();
        
  if (DEBUG || runtime_debug) {Serial.println("in follow wall, distance = ");}
  if (DEBUG || runtime_debug) { Serial.println(distance);}

  if (DEBUG || runtime_debug) {Serial.println("opposite distance = ");}
  if (DEBUG || runtime_debug) { Serial.println(oppositeDistance);} 

//  follow wall



if(oppositeDistance < dangerDistanceOpposite - 10) {
    if (DEBUG || runtime_debug) {Serial.println("way too close, move away");}
    if(isLeft) {
      strongRight();
    } else {
      strongLeft();
    }       
} else if(oppositeDistance < dangerDistanceOpposite) {
    if (DEBUG || runtime_debug) {Serial.println("follow wall, ease away from it");}
    if(isLeft) {
      easeRight();
      //strongRight();
    } else {
      easeLeft();
    }       
} else if(oppositeDistance <  dangerDistanceOpposite + 22) {  
    if (DEBUG || runtime_debug) {Serial.println("follow wall, ease back into it");}
    if(isLeft) {
      easeLeft(); 
    } else {
      easeRight();
    } 
  } else if((oppositeDistance > dangerDistanceOpposite)) {    
    // too far away from wall
    if (DEBUG || runtime_debug) {Serial.println("follow wall, strong turn into it");}
    if(isLeft) {
      strongLeft();
    } else {
      strongRight();
    }
  }
}



void easeRight() {  
  if (DEBUG || runtime_debug) {Serial.print("in easeRight");}
  analogWrite(leftMotor_en, 225);
  analogWrite(rightMotor_en, 210);  
  
  digitalWrite(leftMotor_1, LOW);
  digitalWrite(leftMotor_2, HIGH); 
 
  digitalWrite(rightMotor_1, HIGH);
  digitalWrite(rightMotor_2,  LOW);     
}

void strongRight() {  
  if (DEBUG || runtime_debug) {Serial.print("in strongRight");}
  analogWrite(leftMotor_en, 225);
  analogWrite(rightMotor_en, 185);  
  
  digitalWrite(leftMotor_1, LOW);
  digitalWrite(leftMotor_2, HIGH); 
 
  digitalWrite(rightMotor_1, HIGH);
  digitalWrite(rightMotor_2,  LOW);      
}

void easeLeft() {  
  if (DEBUG || runtime_debug) {Serial.print("in easeLeft"); }
  analogWrite(leftMotor_en, 210);
  analogWrite(rightMotor_en, 255);  
  
  digitalWrite(leftMotor_1, LOW);
  digitalWrite(leftMotor_2, HIGH); 
 
  digitalWrite(rightMotor_1, HIGH);
  digitalWrite(rightMotor_2,  LOW);      
}


void strongLeft() {  
  if (DEBUG || runtime_debug) {Serial.print("in strongLeft");  }
  analogWrite(leftMotor_en, 175);
  analogWrite(rightMotor_en, 255);
  
  digitalWrite(leftMotor_1, LOW);
  digitalWrite(leftMotor_2, HIGH); 
 
  digitalWrite(rightMotor_1, HIGH);
  digitalWrite(rightMotor_2,  LOW);   
}



