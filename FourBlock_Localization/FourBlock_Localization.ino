#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#include <NewPing.h>
#include <Servo.h>
#define LED_STRING_PIN  50
//#define NUMPIXELS 5


#define address 0x1E //001 1110b(0x3C>>1), I2C 7bit address of HMC5883
#define MagnetcDeclination 4.43 //Shanghai
#define CalThreshold 0
int offsetX = 447;
int offsetY = -842;
int offsetZ = 0;

#define e1A 18 // green right side
#define e1B 19 // red right side

#define drive1_1 2
#define drive1_2 3
#define drive2_1 4
#define drive2_2 5

#define trig1 22
#define trig2 23
#define echof 24
#define echol 25
#define echor 26
#define echob 27
#define echobot 28

#define IR 29

#define bigSpin 8
#define smallSpin 10

#define turn -41

int fakewall; 
double multi;

int aState;
int aLastState;

int resultIR[4];
int resultUS[4];
int ReadCount=0;

int Stopflag = 1;
int Turnflag = 0;

int totalReadings = 0; 
int irReading;

volatile int dist = 104; //for 12"; // 26 for 3"
volatile int CountRight = 0;

int OldCount= 0;
long duration;
long distance = 11;
long ref;

double leftReading;
double rightReading;
double frontReading;
double botReading;
double Kp;

bool startflag = false;
char command;

int heading = 0;
char headings[4] = {'w','d','s','a'};

int* SweepResult;
int* clickResult;
int* BlockPosition;
int* TopResult;
int StartingEdge =0;
int FallingEdge = 0;
int Location =0;
int Average;

int stepB = 140;
int stepS = 100;

int delayval = 100; // delay for half a second
int color;

NewPing LSensor(trig1, echol, 200);
NewPing RSensor(trig1, echor, 200);
NewPing FSensor(trig2, echof, 200);
NewPing BotSensor(trig2, echobot, 200);

Servo bigS;  
Servo smallS;

int blockDist = 240;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(5, LED_STRING_PIN, NEO_GRB + NEO_KHZ800);

//-------------------------------------End of Libraries and Variables ----------------------------------------

void setup() {
  
  Serial2.begin(9600); //To Bluetooth
  pixels.begin(); // This initializes the NeoPixel library.

  pinMode (drive1_1, OUTPUT); //Motors
  pinMode (drive1_2, OUTPUT);
  pinMode (drive2_1, OUTPUT);
  pinMode (drive2_2, OUTPUT);
  pinMode (e1A, INPUT); //Encoders
  pinMode (e1B, INPUT);
  pinMode (trig1, OUTPUT); //Front Ultrasonic
  pinMode (trig2, OUTPUT); //Side Ultrasonic
  pinMode (echof, INPUT); //Echos
  pinMode (echol, INPUT);
  pinMode (echor, INPUT);
  pinMode (echob, INPUT);
  pinMode (IR, INPUT); //IR
  pinMode (13,OUTPUT); //LED

  bigS.attach(bigSpin);  //4 bar linkage
  smallS.attach(smallSpin); //Gripper

  bigS.write(140); //0 is fully open 180 is fully closed
  smallS.write(100); // 20 is open 130ish is closed
  
  //attach interrupt pins to the 5th(pin 18) and 2nd pins (Pin 21)
  attachInterrupt(digitalPinToInterrupt(e1B),EncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(e1A),EncoderRight, CHANGE);

  //Initialize Serial and I2C communications
  Serial.begin(9600);
  Wire.begin();
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883r
  Wire.write(0x00); //select configuration register A
  Wire.write(0x70); //0111 0000b configuration
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //set continuous measurement mode:0x00,single-measurement mode:0x01
  Wire.endTransmission();
  
}

void loop() {

    //Wait for commands from Matlab and perform once
    
    if (Serial2.available()) {
      command = Serial2.read();
    }

    //Calibrate Compas
    if (command == 'c'){
      calibrateMag();
    }

    //Orient towards true north
    if (command == 't') { 
      correctToZero();
    }

    //Grab Localization readings
    if (command == 's'){
      correctToZero();
      
      char colorString[] = "GGGGG";
      Lit(colorString);
      
      StartupReading();
      PrintArray();
    }

    //Go Forward
    if (command == 'w'){
      while(Stopflag != 1){
      driveforward(545, 1);
      }
      CountRight = 0;
      //Serial.println(heading);
      //Serial.println(headings[heading]);
      delay(200);
      Serial2.write(FrontRead());
    }

    //
    if (command == 'r'){
      motorRev(); 
      Serial2.write(1);
    }

    //Turn Right
    if (command == 'd'){
      turnright();
      Serial2.write(FrontRead());
    }

    //Turn Left
    if (command == 'a'){
      turnleft();
      Serial2.write(FrontRead());
    }

    //Engage Gripper
    if (command == 'g'){
      blockSense();
      Serial2.write(1);
    }

    //Drop Block
    if (command == 'v'){
      motorRev();
      delay(500);
      motorStop();
      dropBlock();
      Serial2.write(1);
    }

    //
    if (command == 'f'){
      ScanBlock(120);
    }

    //-----------------------Light Sequencess---------------------------------
    if (command == 'l'){
      char colorString[] = "BBBBB";
      Lit(colorString);
      Serial2.write(1);
      //During correct to zero()
    }
    if (command == 'm'){
      char colorString[] = "RRRRR";
      Lit(colorString);
      Serial2.write(1);
      //during localization turns
    }
    if (command == 'n'){
      char colorString[] = "GGGGG";
      Lit(colorString);
      Serial2.write(1);
      //localized
    }
    if (command == 'o'){
      char colorString[] = "YYYYY";
      Lit(colorString);
      Serial2.write(1);
      //reached LZ
    }
    if (command == 'p'){
      char colorString[] = "GGGGG";
      Lit(colorString);
      Serial2.write(1);
      //picked up block
    }
    if (command == 'q'){
      char colorString[] = "MMMMM";
      Lit(colorString);
      Serial2.write(1);
      //blocked dropped
    }
    if (command == 'x'){
      rave();
      Serial2.write(1);
      //blocked dropped
    }
    //-----------------------End of Light Sequences------------------------------
    
    command = 0; //Refresh command, wait for a new one
    Stopflag = 0;
    /*if (Stopflag == 1){
      Stopflag = 0;
      delay(1000);
    }*/
  //}
}


void driveforward(int df_fakewall, double df_multi){
  //Core Function to drive forward a certain distance measured in inches
  //Steer left or right proportional to the central offset error of the rover
  
  delayMicroseconds(2500);
  frontReading = FSensor.ping();
  
  leftReading = LSensor.ping();
  
  if (leftReading == 0) {
    leftReading = 500;
  }
  
  //Serial.print("Left:");
  //Serial.println(leftReading);
  
  delayMicroseconds(2500);
  rightReading = RSensor.ping();
  
  if (rightReading == 0) {
    rightReading = 500;
  }
  
  //Serial.print("Right:");
  //Serial.println(rightReading);
  
  if (frontReading  < 270 && frontReading != 0) {  // emergency stop case, wall present directly ahead
    motorStop();
    Stopflag = 1;
    
  }
  else {
    if (leftReading > 1900 && rightReading > 1900 && Stopflag != 1) {
      //Special case where you see opening in all 4 directions. Just drive ahead
      Motor1(1, 96*df_multi);
      Motor2(1, 100*df_multi);
    }
    else if (leftReading > rightReading && Stopflag != 1) {
      if (rightReading > df_fakewall) {
        Kp = 100 * (1 - (rightReading - df_fakewall) / rightReading);
        steerRight(Kp, df_multi);
      }
      else {
        Kp = 100 * (1 - (leftReading - rightReading) / leftReading);
        steerLeft(Kp, df_multi);
      }
    }
    else if (rightReading > leftReading  && Stopflag != 1) {
      if (leftReading > df_fakewall) {
        Kp = 100 * (1 - (leftReading - df_fakewall) / leftReading);
        steerLeft(Kp, df_multi);
       }
      else {
        Kp = 100 * (1 - (rightReading - leftReading) / rightReading);
        steerRight(Kp, df_multi);
      }
    }
  }
}


void motorRev() { 
  //moves rover back
  Motor1(2, 50);
  Motor2(2, 50);
}


void motorStop() { 
  //stops rover
  Motor1(0, 100);
  Motor2(0, 100);
}


void steerRight(double diff, double sR_multi) { 
  //steers to the right 
  Motor1(1, diff*sR_multi);
  Motor2(1, 100*sR_multi);
}


void steerLeft(double diff, double sL_multi) { 
  //steers to the left
  Motor1(1, 100*sL_multi);
  Motor2(1, diff*sL_multi);
}


void Motor1(int dir, int spd) { //Right Motor control
  //1 is forward, 2 is backwards, 0 is stop
  int val = spd * 255 / 100;

  if (dir == 0) {
    digitalWrite(drive1_1, LOW);
    digitalWrite(drive1_2, LOW);
  }
  else if (dir == 1) {
    analogWrite(drive1_1, val);
    digitalWrite(drive1_2, LOW);
  }
  else if (dir == 2) {
    digitalWrite(drive1_1, LOW);
    analogWrite(drive1_2, val);
  }
}


void Motor2(int dir, int spd) {
  //1 is forward, 2 is backwards, 0 is stop
  int val = spd * 255 / 100;

  if (dir == 0) {
    digitalWrite(drive2_1, LOW);
    digitalWrite(drive2_2, LOW);
  }
  else if (dir == 1) {
    analogWrite(drive2_1, val);
    digitalWrite(drive2_2, LOW);
  }
  else if (dir == 2) {
    digitalWrite(drive2_1, LOW);
    analogWrite(drive2_2, val);
  }
}


void turnright() {
  CountRight = 0;
  while (CountRight > turn){
    Motor1(2, 100);
    Motor2(1, 100); 
  }
    CountRight = 0;
    motorStop();
    updateHeading(1);
}


void turnleft() {
  CountRight = 0;
  while (CountRight < abs(turn)){
    Motor1(1,100);
    Motor2(2,100);
  }
  CountRight = 0;
  motorStop();
  updateHeading(0);
}


void correctToZero(){
  int randHeading = getheading() / 110;
  //Serial.println(getheading());
  //Serial.println(randHeading);

  if (randHeading == 0) {
    turnleft();    
  }

  else if (randHeading == 1) {
    turnleft();
    delay(200);
    turnleft();
  }

  else if (randHeading == 2) {
    turnright();
  }
  
}


void StartupReading(){
  totalReadings = 0;
  for (int i=1; i<=4; i++){
    LocalRead();  //Read IR, Read Front Ultrasonic
    delay(500);
    turnright();  //Rotate 90
  }
}


void LocalRead(){
  // Read IR, if black send 1, if white send 2
  totalReadings ++;
  //Serial.println(totalReadings);
  irReading = digitalRead(IR);
  if (irReading == 0){
    resultIR[totalReadings-1] = 2;
  }
  else {
    resultIR[totalReadings-1] = 1;
  }
  // Read Ultrasonic, if Wall is present send 1, if wall isn't present send 0
  frontReading = FSensor.ping();
  //Serial.println(frontReading);
  while (frontReading == 0)
  {
    frontReading=FSensor.ping();
    //Serial.println(frontReading);
  }
  if (frontReading < 600 && frontReading !=0){
    resultUS[totalReadings-1] = 1;
  }
  else {
    resultUS[totalReadings-1] = 0;
  }
}


void PrintArray(){
  for (int i=0;i<totalReadings;i++)
  {
    Serial2.write(resultIR[i]);
  }
  for (int i=0;i<totalReadings;i++)
  {
    Serial2.write(resultUS[i]);
  }
}


int FrontRead(){
  frontReading = FSensor.ping();
  while (frontReading == 0){
  frontReading=FSensor.ping();
  }
  if (frontReading < 600 && frontReading !=0){
    return 1;
  }
  else {
    return 0;
  }
}


void EncoderRight(){
   aState = digitalRead(e1A); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(e1B) != aState) { 
       CountRight ++;
     } else {
       CountRight --;
     }
   } 
   aLastState = aState; // Updates the previous state of the outputA with the current state
   // Move in incremental steps to debug and track motion
   
   if (CountRight > dist){
    Stopflag = 1;
    motorStop();
    CountRight = 0;
    }
}


int pickBlock() {

  for(stepB; stepB >= 80; stepB--) {        //extend gripper out of robot body
    bigS.write(stepB);
    delay(10);
  }
  delay(50);
  for (stepS; stepS >= 20; stepS--) {       //fully open gripper claws
    smallS.write(stepS);
    delay(10);
  }
  delay(50);
  for(stepB; stepB >= 50; stepB--) {        //extend gripper to be around block          
    bigS.write(stepB);
    delay(10);
  }
  delay(50);
  for (stepS; stepS <= 130; stepS++) {       //fully close gripper claws
    smallS.write(stepS);
    delay(10);
  }
 delay(500);
 for (stepB; stepB <=140; stepB++) {        //fully retract gripper
    bigS.write(stepB);
    delay(10);
 }
  Serial2.write(1); 
}


int dropBlock () {
  for(stepB; stepB >= 50; stepB--) {        //fully extend gripper
    bigS.write(stepB);
    delay(10);
  }
  delay(50);
  for (stepS; stepS >= 70; stepS--) {       //slightly open gripper claws
    smallS.write(stepS);
    delay(10);
  }
 delay(50);
 for (stepB; stepB <=80; stepB++) {        //partly retract gripper
    bigS.write(stepB);
    delay(10);
 } 
 delay(50); 
  for (stepS; stepS <=100; stepS++) {        //fully close gripper claws
    smallS.write(stepS);
    delay(10);
 }
 delay(50);
  for (stepB; stepB <=140; stepB++) {        //fully retract gripper
    bigS.write(stepB);
    delay(10);
 }

 motorRev();
 delay(700);
 motorStop();

 Serial2.write(1);

}


char updateHeading (int turning){
  // case if turning right
  if (turning == 1){
    if (heading !=3 ){
      heading++;
    }
    else {
      heading = 0;
    }
  }
  // case if turning left
  else{
    if (heading != 0){
      heading --;
    }
    else{
      heading = 3;
    }
  }
}


//--------------------------- SIMPLE SCANBLOCK-------------------------------//

void blockSense () {

  do {
    CountRight = -26;
    driveforward(545, 1);
    //Motor1(1,50);
    //Motor2(1,50);
 
    botReading = BotSensor.ping();
    //Serial.println(botReading);
    if (abs(botReading - blockDist) <= 10) {
      motorStop();
      pickBlock();
      return;    
    }
    
  } while (1);

}


//-------------------COMPLEX SCANBLOCK, Not used for Milestone 2----------------------//

int ScanBlock (int Angle){
  Angle = Angle/2.25; // convert to degrees
  // sweep small angle infront of robot, take readings
  CountRight = 0;
  totalReadings = 0;
  // turn to starting position
  while (CountRight < (Angle/2)){
    Motor1(1,100);
    Motor2(2,100);
  }
  motorStop();
  OldCount = CountRight;
  // turn sweep of desired angle and take readings
  while (abs((CountRight)-(OldCount)) < Angle){
    totalReadings++;
   long SumReading = 0;
    //Allocate memory for Base sensor, Top Sensor, Encoder clicks and edges as you take readings
    //
    if (SweepResult != 0){
      SweepResult = (int*)realloc(SweepResult,totalReadings*sizeof(int));
      TopResult = (int*)realloc(TopResult,totalReadings*sizeof(int));
      clickResult = (int*)realloc(clickResult,totalReadings*sizeof(int));
      BlockPosition = (int*)realloc(BlockPosition,totalReadings*sizeof(int));
      
    }
    else{
      TopResult = (int*)malloc(totalReadings*sizeof(int));
      SweepResult = (int*)malloc(totalReadings*sizeof(int));
      clickResult = (int*)malloc(totalReadings*sizeof(int));
      BlockPosition = (int*)malloc(totalReadings*sizeof(int));
    }
    // get values for Top and Bottom sensors and encoders
   frontReading = FSensor.ping();
  for (int i=0; i<10; i++){
    motorStop();
    delay(100);
    frontReading = FSensor.ping();
      while(frontReading == 0){
        motorStop();
        frontReading = FSensor.ping();
      }
      SumReading = frontReading + SumReading;
    }
  Serial.print("Top Reading Average:");
  Serial.println(SumReading/10);
  TopResult[totalReadings - 1] = SumReading/10;
  SumReading = 0;
  botReading = BotSensor.ping();
  for(int i=0; i<10; i++){
    while(botReading == 0){
      motorStop();
      botReading = BotSensor.ping();
    }
    SumReading = botReading+SumReading;
   }
    Serial.print("Bottom Reading Average:");
    Serial.println(SumReading/10);
    SweepResult[totalReadings - 1] = SumReading/10;
    clickResult[totalReadings-1] = CountRight;
    
    Motor1(2,50);
    Motor2(1,50);
    delay(200); // THIS DELAY LIMITS THE NUMBER OF SAMPLES TO TAKE, IF TOO MANY ARE TAKEN YOU RUN OUT OF MEMORY - hypothesis
    //Serial.println(CountRight);
 //------------------------------- TEST CASE STOPS IMMEDIATELY WHEN FINDING THE BLOCK -----------------------------------------------------------
    if (abs(SweepResult[totalReadings - 2] - SweepResult[totalReadings-3] )>300 && abs(SweepResult[totalReadings-2] - TopResult[totalReadings-2]) > 500 && totalReadings >3){
      motorStop();
      Serial.println ("BLOCK FOUND IN FRONT");
      return clickResult[totalReadings-1];
    }
// ------------------------------------------------------------------------------------------------------------------------------------------------
    if (abs((CountRight)-(OldCount)) < Angle){
      motorStop();
    }
  }
  motorStop();
  
  // turn back to starting position
  delay (500);
  while (CountRight < 0) { // SHOULD TECHNICALLY BE 0 SOMTHING ISNT WORKING RIGHT??
    Motor1(1,100);
    Motor2(2,100);
  }
  CountRight = 0; // PUT IN TO EXPLAIN OFFSET WHEN TURNING BACK TO ORIGIN
  motorStop();


// ---------------- Light function ---------------------- //


void Lit(char InputString[]){

  for(int i=0;i<5;i++){
    if(InputString[i] == 'R'){
      pixels.setPixelColor(i,pixels.Color(100,0,0)); //red
    }
    else if(InputString[i] == 'G'){
      pixels.setPixelColor(i,pixels.Color(0,100,0)); //green
    }
    else if(InputString[i] == 'B'){
      pixels.setPixelColor(i,pixels.Color(0,0,100)); //blue
    }
    else if(InputString[i] == 'Y'){
      pixels.setPixelColor(i,pixels.Color(80,30,0)); //yellow
    }
    else if(InputString[i] == 'P'){
      pixels.setPixelColor(i,pixels.Color(94,100,202)); //purple
    }
    else if(InputString[i] == 'M'){
      pixels.setPixelColor(i,pixels.Color(100,0,80));  //magenta
    }
    else{
      pixels.setPixelColor(i,pixels.Color(40,40,40)); //else white
    }
   
    //pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    //green (0,80,0)
    //red (80,0,0)
    //blue (0,0,80)
    //yellow (80,80,0)
  }
  pixels.show();
}

void sensortest(){
  
}


//------------- Compass functions----------------//

int getheading(){
  int x,y,z; //triple axis data
  getRawData(&x,&y,&z);
  
  //Print out values of each axis
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.print(z);
  Serial.print(" angle(x,y): ");
  Serial.println(calculateHeading(&x,&y,&z));
  return (calculateHeading(&x,&y,&z));
  //delay(250);
}

void getRawData(int* x ,int* y,int* z)
{
  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    *x = Wire.read()<<8; //X msb
    *x |= Wire.read(); //X lsb
    *z = Wire.read()<<8; //Z msb
    *z |= Wire.read(); //Z lsb
    *y = Wire.read()<<8; //Y msb
    *y |= Wire.read(); //Y lsb
  }
}

int calculateHeading(int* x ,int* y,int* z)
{
  float headingRadians = atan2((double)((*y)-offsetY),(double)((*x)-offsetX));
  // Correct for when signs are reversed.
  if(headingRadians < 0)
    headingRadians += 2*PI;
    
  int headingDegrees = headingRadians * 180/M_PI;
  headingDegrees += MagnetcDeclination; //the magnetc-declination angle 
  
  // Check for wrap due to addition of declination.
  if(headingDegrees > 360)
    headingDegrees -= 360;
  
  return headingDegrees;
}

void calibrateMag()
{
  int x,y,z; //triple axis data
  int xMax, xMin, yMax, yMin, zMax, zMin;
  //initialize the variables
  getRawData(&x,&y,&z);  
  xMax=xMin=x;
  yMax=yMin=y;
  zMax=zMin=z;
  offsetX = offsetY = offsetZ = 0;
  
  //Serial2.write(6);
  //Serial.println("Please turn your device around in 20 seconds");
  
  for(int i=0;i<200;i++)
  {
    getRawData(&x,&y,&z);
    //get Max and Min
    // this routine will capture the max and min values of the mag X, Y, and Z data while the
    // compass is being rotated 360 degrees through the level plane and the upright plane.
    // i.e. horizontal and vertical circles.
    // This function should be invoked while making continuous measurements on the magnetometers
    if (x > xMax)
      xMax = x;
    if (x < xMin )
      xMin = x;
    if(y > yMax )
      yMax = y;
    if(y < yMin )
      yMin = y;
    if(z > zMax )
      zMax = z;
    if(z < zMin )
      zMin = z;
      
    delay(100);

    
    if(i%10 == 0)
    {
      Serial.print(xMax);
      Serial.print(" ");
      Serial.println(xMin);
    }
    
  }
  //compute offsets
  if(abs(xMax - xMin) > CalThreshold )
    offsetX = (xMax + xMin)/2;
  if(abs(yMax - yMin) > CalThreshold )
    offsetY = (yMax + yMin)/2;
  if(abs(zMax - zMin) > CalThreshold )
    offsetZ = (zMax +zMin)/2;
  
  Serial.print("offsetX:");
  Serial.print("");
  Serial.print(offsetX);
  Serial.print(" offsetY:");
  Serial.print("");
  Serial.print(offsetY);
  Serial.print(" offsetZ:");
  Serial.print("");
  Serial.println(offsetZ);
  
  delay(1000);

  Serial.println("done");
}


void rave(){

  CountRight = -10000;
  Motor1(1, 100);
  Motor2(2, 100);
  
    for(int j=0; j<1000; j++){
      for(int i=0; i<5; i++) {
        int C1 = random(120);
        int C2 = random(120);
        int C3 = random(120);
        pixels.setPixelColor(i,pixels.Color(C1,C2,C3)); //random for all rbg value
        pixels.show();
      }
      delay(150);
    }
      
      delay(100);
}


