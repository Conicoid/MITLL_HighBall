//Team High Ball
//Altitude Control System Software


#include <IridiumSBD.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define GPSSerial Serial1
#define IridiumSerial Serial3
#define DIAGNOSTICS true // Change this to see diagnostics
#define RING_PIN 53
bool ring = false;
SoftwareSerial debugSerial(45, 47);

int balLevel = 70; //seconds ballast can drop for
//control variables
float targetAlt = 10000;
float currentAlt = 0;
float averageVelocity = 0;
float temp = 0; //payload interior temperature
float powerStatus = 0;
int powerSetting = 0;
bool balVenting = false;;
bool heVenting = false;
bool ascending = false;
bool descending = false;
bool hovering = false;
bool manuelOverride = true;

Servo heServo;
Servo balServo;


//autonomous logic variables
float altData[20]; //circle buffer used to hold altitude measurements
int altIdx; //index of the next entry in the altitude buffer to write to
int altLen = 20;
float vData[20];  //cirlce buffer used to hold velocity estimates based off altitude measurements
int vIdx; //index of next entry in velocity buffer to write to
int vLen = 20;
float acceleration = 0; //approximate acceleration


//below are variables used to configure the autonomous controls during the flight
int innerAltRange = 500; //initial inner altitude range set to 500m
int outerAltRange = 1000; //initial outer altitude range set to 1000m
int heOpenThreshold = 3; 
int balOpenThreshold = 3; 
int hoverThreshold = 2;



//location variables
float latitude = 0; 
float longitude = 0;
char lat = 'N';
char lon = 'W';
String latString = "HELLO";
String lonString = "HELLO";

//input pins
int heClosed = 49;
int heOpen = 51;
int powerSensor = A5;


//timers used to regulate various update rates
uint32_t timer = millis();
uint32_t altTimer = millis();
uint32_t GPStimer = millis();
uint32_t updateTimer = millis();
int updateRate = 20; //update rate in seconds


//hardware objects with related variables
Adafruit_GPS GPS(&GPSSerial);
IridiumSBD modem(IridiumSerial, -1, RING_PIN);
Adafruit_BMP280 bmp; // I2C

float seaLevelPressure = 1011;
uint8_t buffer[200] = 
{ 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89 };

////Helium Functions
//helium valve failsafe variables
uint32_t heTimer = millis();
int valveFailSafe = 2000; 
//control functions
void openHe() {
  //if (heVenting = true) {return;} //check if valve already open
  heVenting = true;
  int reading = 1;
  reading = digitalRead(heOpen);
  heTimer = millis();
  if (reading) { //if valve already open don't run (0 indicates an open valve)
    heServo.write(50);
    reading = digitalRead(heOpen);
    while(millis()-heTimer < valveFailSafe && reading){ //if reading is 0 then valve is open and the timer is a failsafe
      if (heTimer > millis()) {heTimer = millis();}
      reading = digitalRead(heOpen);
    }
    heServo.write(89);
  }
}

void closeHe() {
  //if (heVenting = false) {return;} //check if valve already closed
  heVenting = false;
  int reading = 1;
  reading = digitalRead(heClosed);
  heTimer = millis();
  if (reading) { //if valve already closed don't run (0 indicates an open valve)
    heServo.write(150);
    reading = digitalRead(heClosed);
    while(millis()-heTimer < valveFailSafe && reading){ //if reading is 0 then valve is closed and the timer is a failsafe
      if (heTimer > millis()) {heTimer = millis();}
      reading = digitalRead(heClosed);
    }
    heServo.write(89);
  }
}

////Ballast Functions
uint32_t balTimer = millis();
int balPeriod = 0;
uint32_t timeSinceLastDrop = millis();
void dropBal(int seconds) {
  if (!balVenting){
    balLevel -= seconds;
    balPeriod = seconds * 1000;
    balVenting = true;
    balTimer = millis();
    balServo.write(60);
  }
}

void closeBal() {
  if (millis() - balTimer > balPeriod && balVenting) {
    balServo.write(10);
    balVenting = false;
    timeSinceLastDrop = millis();
  }
}


////Flight Termination
int killSwitch = 19;
int parachuteRelease = 20;
void terminate() {
  manuelOverride = true;
  updateRate = 10;
  delay(1000);
  digitalWrite(killSwitch, HIGH);
  delay(2000);
  digitalWrite(killSwitch,LOW);
  delay(1000);
  digitalWrite(parachuteRelease, HIGH);
  delay(1000);
  digitalWrite(parachuteRelease, LOW);
}


////Update Altitude and Calculate Average Velocity
int altUpdateRate = 5; // update rate for altitude sensor in seconds
//readBMP handles reading the altitude data as well as calculating velcotiy and acceleration
void readBMP() {
if (millis() - altTimer > altUpdateRate * 1000) {
  int lastAltIdx, lastVIdx;
  //reset circle buffers and set velocity/acceleration calculation variables
  if (altIdx == altLen || altIdx == 0) {
    altIdx = 0;
    lastAltIdx = altLen - 1;
  }
  else{
    lastAltIdx = altIdx - 1;
  }
  if (vIdx == vLen || vIdx == 0) {
    vIdx = 0;
    lastVIdx = vLen - 1;
  } 
  else {
    lastVIdx = altIdx - 1;
  }
  
  //read altitude and calculate time since last reading
  altData[altIdx] = bmp.readAltitude(seaLevelPressure);
  float deltaT = millis() - altTimer;
  deltaT /= 1000; //convert to seconds
  altTimer = millis(); //reset timer for next reading
  
  vData[vIdx] = (altData[altIdx] - altData[lastAltIdx])/(deltaT);
  acceleration = (vData[vIdx] - vData[lastVIdx])/(deltaT);

  //calculate average velocity
  averageVelocity = 0;
  averageVelocity += vData[vIdx];
  averageVelocity += vData[lastVIdx];
  if (lastVIdx == 0) {averageVelocity += vData[vLen-1];}
  else {averageVelocity += vData[lastVIdx-1];}
  averageVelocity /= 3; //average velocity is now set to the average of the last three velocities

    //process new velocity reading
  if (averageVelocity > 0) {
    ascending = true;
    descending = false;
  }
  else if (averageVelocity < 0) {
    descending = true;
    ascending = false;
  }
  if (averageVelocity < hoverThreshold && averageVelocity > hoverThreshold * -1) {hovering = true;}
  else {hovering = false;}
  
  //update global variables
  currentAlt = altData[altIdx];
  temp = bmp.readTemperature();
  altIdx++;
  vIdx++;
}
}



void readGPS() {

  // read data from the GPS in the 'main loop'
  char c = GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - GPStimer > 5000) {
    GPStimer = millis(); // reset the timer
    lat = GPS.lat;
    lon = GPS.lon;
    String tempLat;
    String tempLon;
    latitude = GPS.latitude;
    longitude = GPS.longitude;
    //need to format latitude and longitude strings
    //current format is XXYY.ZZZZC where X is degrees, Y is minutes, Z is decimal minutes, and C is either lat or lon
    tempLat = String(latitude);
    tempLon = String(longitude);
    float seconds = latitude - floor(latitude);
    seconds *= 60;
    latString = tempLat.substring(0,2) + "*" +tempLat.substring(2,4) + "'" + String(seconds) + "\"" + " " + lat; 
    seconds = longitude - floor(longitude);
    seconds *= 60;
    lonString = tempLon.substring(0,2) + "*" +tempLon.substring(2,4) + "'" + String(seconds) + "\"" + " " + lon;

  } 
}


////RockBLOCK Message Functions
int messageCount = 1;
void sendStatusUpdate(){
  int err;
  size_t bufferSize = sizeof(buffer);
  String updateMess = String("Location: ")+latString+" "+lonString+" Current Alt: "+String(currentAlt)+" Target Atl: "+String(targetAlt)+" He Open: "+String(heVenting)+" Bal Level: "+String(balLevel)+ "Voltage: " +String(powerStatus) + " Temp: " + String(temp);
  
  Serial.println(updateMess);
  char outputMess[200];
  updateMess.toCharArray(outputMess, updateMess.length());
  err = modem.sendReceiveSBDText(outputMess, buffer, bufferSize);      
  if (err != ISBD_SUCCESS)
  {
    Serial.print("sendReceiveSBD* failed: error ");
    Serial.println(err);
  }
  Serial.println(bufferSize);
  if (bufferSize > 0) {
    decodeMessage(bufferSize);
  }
  
}

/////incoming iridium message decoder
void decodeMessage(int bufferSize) {
//do {
  String message = buffer;
  char messageNum = message.charAt(0);// used for ack message
  message = message.substring(1, bufferSize);
  int tempAlt;
  bool decoded = true;
  switch(message.charAt(1)) { 
    case 'A': //specifiying new target altitude
    //parse the message for the altitude data
    //the toFloat() char[] method only works for a single decimal place
        tempAlt = message.substring(2).toInt();
        targetAlt = tempAlt;
      break;
    case 'P': //power setting
      powerSetting = message.substring(2).toInt();
      break;
    case 'O': //manual valve open command
      if (message.charAt(2) == 'H') {openHe();}
      else if (message.charAt(2) == 'B') {
        tempAlt = message.substring(3).toInt();
        dropBal(tempAlt);  
      }
      manuelOverride = true; //turn off autonomous
      break;
    case 'C': //manual valve close command
      if (message.charAt(2) == 'H') {closeHe();}
      else if (message.charAt(2) == 'B') {closeBal();}
      manuelOverride = true; //turn off autonomous
      break;
    case 'D': //data dump command (will be somewhat complicated)
      break;
    case 'U': //set message update rate
      //use tempAlt to hold new update rate
      tempAlt = message.substring(2).toInt();
      updateRate = tempAlt;
      break;
    case 'E': //reengage autonomous mode
      manuelOverride = false;
      break;
    case 'F': //force an update message
      sendStatusUpdate();
      break;
    case 'K': //kill command
      if (message.charAt(2) == 'I') {
        if (message.charAt(3) == 'L') {terminate();}
      }
      break;
    case 'S': //set autonomous control logic variables
      switch(message.charAt(2)){ //use another switch to decide which variable to set
        case 'R': //setting inner or outer target altitude range
          if (message.charAt(3) == 'I') {
            innerAltRange = message.substring(4).toInt();
          }
          else if (message.charAt(3) == 'O') {
            outerAltRange = message.substring(4).toInt();
          }
          break;
        case 'T': //updating a velocity threshold value
          if (message.charAt(3) == 'H'){ //setting helium threshold
            heOpenThreshold = message.substring(4).toInt();
          }
          else if (message.charAt(3) == 'B') { //setting ballast threshold
            balOpenThreshold = message.substring(4).toInt();
          }
          break;
        case 'H': //set hover threshold
          hoverThreshold = message.substring(3).toInt();
          break;
      } //end of inner switch
      default: 
        decoded = false;
      break; //end of set autonomous logic case   
  } //end of decode loop
  //time to send acknowledgement
  if (decoded) { //only ack if a valid message was received
    char ackMess[5];
    ackMess[0] = 'A';
    ackMess[1] = 'c';
    ackMess[2] = 'k';
    ackMess[3] = messageNum;
    ackMess[4] = message.charAt(1);
    size_t bufferSize = sizeof(buffer);
    int err = modem.sendReceiveSBDText(ackMess, buffer, bufferSize); 
  }
//} while(bufferSize > 0);     

}

////Power Status Functions
void lowPowerMode() {
  updateRate = 600; //update every 10 minutes
  manuelOverride = true; //turn off autonomous valve control 
}

bool LPFLAG = false;
int lowPowerLevel = 0;
void checkPower() {
  powerStatus = analogRead(powerSensor);
  powerStatus = powerStatus * 5 /1023; //convert to voltage
  if (powerStatus < 2) {LPFLAG = true;}
  //if (powerStatus < lowPowerLevel) {lowPowerMode();} 
  
}

int risingTooFast = 5;
void autonomousControl() {
  //check for range violations
  if (millis() - timeSinceLastDrop > 20000) { //only check if ballast needs to be dropped every 20 seconds to preserve ballast and allow the weight change to fully take affect
    if (currentAlt < targetAlt-outerAltRange) { //if the balloon is currently below the target altitude's outer range
      if (!ascending || averageVelocity < balOpenThreshold) {
        dropBal(5);
      }
    }
    else if (currentAlt < targetAlt - innerAltRange) {
      if (!ascending) {dropBal(2);} //the balloon is starting to fall or is ascending too slowly
    }
  }
  
  if (currentAlt < targetAlt) { //if below target but rising too fast openHe
    if (averageVelocity > risingTooFast) {openHe();}
  }
  else if (currentAlt > targetAlt + outerAltRange) { 
    if (!descending || averageVelocity > heOpenThreshold) {openHe();}
  }
  else if (currentAlt > targetAlt + innerAltRange) {
    if (!descending) {openHe();}
  }
  else if (currentAlt < targetAlt + innerAltRange && currentAlt > targetAlt - innerAltRange) { //balloon is within the desired range
    if (!ascending) {closeHe();}
  }
  else {closeHe();}
}

bool fallDetect() {
  if (averageVelocity < -8) {
    terminate();
    return true;
  }
  return false;
}

bool targetReached = false;
void updateStatus() {
  if (currentAlt > 4000 && targetReached == false) {
    manuelOverride = false;
    targetReached = true;
  }
  checkPower(); //check remaining power
  closeBal(); //only closes ballast if specified time has past and ballast is open
  if (millis() - updateTimer > (updateRate * 1000)) { //send update message
    Serial.println("UPDATE SENT");
    sendStatusUpdate();
    updateTimer = millis();
  }
  if (!manuelOverride) {autonomousControl();} //call autonomous controller function

  checkMessage(); //check for any messages
}


uint32_t ringTimer = millis();
int checkAttempts = 0; //variable used to limit checking if not currently able to receive messages
void checkMessage() {
  
  if (checkAttempts < 3 || millis() - ringTimer > 60000 * 2) { //if a message hasn't been received after 3 attempts only check again every 2 minutes
    int err;
    ring = modem.hasRingAsserted();
    size_t bufferSize = sizeof(buffer);
    if (ring || modem.getWaitingMessageCount() > 0) {
      err = modem.sendReceiveSBDText(NULL, buffer, bufferSize);
        
      if (err != ISBD_SUCCESS)
      {
        ringTimer = millis(); //set ring timer to keep track of time spent failing to receive messages
        checkAttempts++;
        Serial.print("sendReceiveSBD* failed: error ");
        Serial.println(err);
      }
      else // success!
      {
    //message format is "<capital letter><number/capital letter>"
    //decode message bellow
        checkAttempts = 0;
        if (bufferSize > 0) {decodeMessage(bufferSize);}
      }
    }
  }
  
}

bool ISBDCallback() { //callback function used during message transmission
  readBMP();
  readGPS();
}

void setup()
{
  delay(2000);
  Serial.begin(115200);
  debugSerial.begin(9600);
//flight termination signals
  pinMode(parachuteRelease, OUTPUT);
  pinMode(killSwitch, OUTPUT);
  digitalWrite(parachuteRelease, LOW);
  digitalWrite(killSwitch, LOW);

//setup valve variables
  heServo.attach(10);
  balServo.attach(9);
  heServo.write(89);
  balServo.write(10);
  pinMode(heOpen,INPUT_PULLUP);
  pinMode(heClosed,INPUT_PULLUP);
  pinMode(powerSensor,INPUT);

  
/////////////////////////BMP Setup
  bmp.begin();
  delay(100);
  float initialValue = bmp.readAltitude(seaLevelPressure);
  //initialize variables
  for (int i = 0; i < altLen; i++) {
    altData[i] = initialValue;
    vData[i] = 0;
  }


//////////////////////////RockBlock Setup
  int signalQuality = -1;

  // Start the serial ports
  Serial.begin(115200);
  while (!Serial);
  Serial.print("HELLO");
  IridiumSerial.begin(19200);

  // Setup the Iridium modem
  modem.setPowerProfile(0);
  if (modem.begin() != ISBD_SUCCESS)
  {
    Serial.println("Couldn't begin modem operations.");
    exit(0);
  }

  // Check the signal quality (optional)
  int err = modem.getSignalQuality(signalQuality);
  if (err != 0)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    exit(1);
  }

  Serial.print("Signal quality is ");
  Serial.println(signalQuality);


////////////////////////GPS setup
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); // 200 mHz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
}


void loop()
{

  //check for timer rap around
  if (altTimer > millis()) {altTimer = millis();}
  if (timer > millis()) { timer = millis();}
  if (GPStimer > millis()) {GPStimer = millis();}
  if (updateTimer > millis()) {updateTimer = millis();}
  if( balTimer > millis()) {balTimer = millis();}
  if (heTimer > millis()) { heTimer = millis();}
  
  readBMP();
  readGPS();
  updateStatus();

}
#if DIAGNOSTICS //the RockBLOCK won't work correctly without these functions defined
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  //Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  //Serial.write(c);
}
#endif

