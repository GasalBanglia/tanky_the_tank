//Library imports
#include <Servo.h>
#include <Stepper.h>

//Pin Settings
#define interruptPin 2
#define pwmTrack1Pin 3
#define pwmTrack2Pin 5
#define dirTrack1Pin 6
#define dirTrack2Pin 7
#define proxTrigPin  8
#define stepCoil4Pin 9
#define stepCoil3Pin 10
#define stepCoil2Pin 11
#define stepCoil1Pin 12
#define proxEchoPin  13

#define MAXSCANS     80
#define STEPS        2038  

//State Variables
bool flailing = false;
bool escaping = false;
bool rotating = false;
int dirTrack1 = 0;
int dirTrack2 = 0;
int pwmTrack1;
int pwmTrack2;
int cruisingSpeed = 100;
int scanningSpeed = 400;
long proxScan[MAXSCANS];

//Objects
Stepper proxStepper(STEPS, 9, 11, 10, 12); //all set up!

void setup() {
  // put your setup code here, to run once:
  //Setup I/O
   pinMode(pwmTrack1Pin, OUTPUT);
   pinMode(pwmTrack2Pin, OUTPUT);
   pinMode(interruptPin, INPUT_PULLUP);
   pinMode(dirTrack1Pin, OUTPUT);
   pinMode(dirTrack2Pin, OUTPUT);

   //attachInterrupt(digitalPinToInterrupt(interruptPin), flail, INPUT);
   changeSpeed(cruisingSpeed);
   updateState();

   //begin serial communication
   //Serial.begin(9600);

   delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
    if (escaping) {
      changeSpeed(0);
      toggleDirection();
      changeSpeed(50);
      delay(3000);
      changeSpeed(0);
      toggleRotate();
      changeSpeed(100);
      delay(2000);
      changeSpeed(0);
      toggleRotate();
      changeSpeed(cruisingSpeed);
      escaping = false;
      //attachInterrupt(digitalPinToInterrupt(interruptPin), flail, INPUT);
    }

    if (detectObstacle()) {
      escaping = true;
    }
    delay(3000);
}

/*
 * Searches scan data for a nearby obstacle.
 * A "nearby obstacle" is represented by 
 * a distance sense of less than 12 inches.
 */
bool detectObstacle() {
  changeSpeed(0);
  int numScans = scanLocale();
  for (int i = 0; i < numScans; i++) {
    if (proxScan[i] < 12) return true;
  }
  changeSpeed(cruisingSpeed);
  return false;
}

/*
 * Directs the stepper to aim the proximity sensor across the tank's front.
 * Takes measurements and stores them in the global array proxScan.
 * Returns the number of measurements.
 * 
 * 
 */
int scanLocale() {
  //declare iterator
  int i;
  //steps between measurements 
  //based on 5.66 steps per angle for the 28BYJ-48 stepper
  int miniStep = 5;
  
  //move MAXSCANS / 2 measurement-steps to the left.
  //This is the start of the scan line.
  proxStepper.setSpeed(10);
  proxStepper.step(-(MAXSCANS / 2) * miniStep);

  //scan the line.
  for (i = 0; i < MAXSCANS; i++) {
    proxScan[i] = readProxSensor('i');
    proxStepper.step(miniStep);
  }
  
  //return to dead center
  proxStepper.step(-(MAXSCANS / 2) * miniStep);

  //return the number of measurements
  return i;
}

/*
 * Takes a distance measurement wherever the 
 * sensor is pointing.
 * For now, the robot SHOULD be facing straight-ahead 
 * of itself.
 * 
 * The function repeatedly calls readProxSensor 
 * until it gets a
 * valid reading.  It tries ten times before giving up 
 * and returning -1.
 */
 int scanAhead() {
  float distance = readProxSensor('i');
  int i = 0;
  while (distance < 0 && i < 10){
    distance = readProxSensor('i'); 
    i++;
  } 
  if (i == 10) return -1;
  else return distance;
 }


/*
 * Prints the scan data in a line to the serial monitor, space-separated
 */
void serialPrintScans(int num) {
  for (int j = 0; j < num; j++) {
    Serial.print(proxScan[j]);
    Serial.print(" ");
  }
  Serial.println();
}

/*
 * Serial Prints the scan data in a way that the Serial Plotter can use.
 */
void serialPlotScans(int num) {
  for (int j = 0; j < num; j++) {
    Serial.println(proxScan[j]);
  }
}

/*
 * Toggles the escaping flag and detaches the interrupt
 */
void flail() {
  flailing = true;
  detachInterrupt(digitalPinToInterrupt(interruptPin));
}

/*
 * Sets the speed the robot is moving at.
 * Both tracks will be given the same PWM output.
 * Valid inputs for speed are integers 0-255.
 * If speed is negative, 0 will be used.
 * If speed is above 255, 255 will be used.
 */
int changeSpeed(int speed) {
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  pwmTrack1 = speed;
  pwmTrack2 = speed;
  updateState();
}

/*
 * Switches the direction both wheels move.
 * Reverses the robot's direction.
 */
void toggleDirection() {
  dirTrack1 = !dirTrack1;
  dirTrack2 = !dirTrack2;
  updateState();
}

/*
 * Toggles rotation of the robot.
 * Call the function again to stop rotation.
 */
void toggleRotate() {
  if (!rotating) {
    dirTrack1 = !dirTrack1;
    rotating = true;
  }
  else {
    dirTrack2 = !dirTrack2;
    rotating = false;
  }
  updateState();
}

void updateState() {
  analogWrite(pwmTrack1Pin, pwmTrack1);
  analogWrite(pwmTrack2Pin, pwmTrack2);
  digitalWrite(dirTrack1Pin, dirTrack1);
  digitalWrite(dirTrack2Pin, dirTrack2);
}

/*
 * Retrieves a distance measurement from the ultrasonic proximity sensor.
 * parameter 'units' specifies the dimensions ('c' = cm, 'i' = in)
 * 
 * If the sensor reading is zero or is above
 */
long readProxSensor(char units) {
  long duration = pulseAndReadProx();
  int i = 0;

  //attempt to obtain a good measurement 
  //(nonzero and within the sensor range)
  //quit after 10 attempts
  //while ((duration > 6024 || duration == 0) && i < 10) {
  while (duration == 0 && i < 10) {
    duration = pulseAndReadProx();
    i++; 
  }

  //lump all long durations as 6024 us (9 feet)
  //this is outside the sensor range, and may not be
  //meaningful.
  if (duration > 6024) duration = 6024;
  
  //'c' for centimeters
  if (units == 'c') return duration / 29 / 2;
  //'i' for inches
  else if (units == 'i') return duration / 74 / 2;
  //'f' for feet
  else if (units == 'f') return duration / 74 / 2 / 12;
}

long pulseAndReadProx() {
  pinMode(proxTrigPin, OUTPUT);
  digitalWrite(proxTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(proxTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(proxTrigPin, LOW);
  pinMode(proxEchoPin, INPUT);
  return pulseIn(proxEchoPin, HIGH);
}
