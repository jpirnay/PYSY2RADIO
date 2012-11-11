///////////////////////////// defines and global variables here /////////////////////////

// PIN MAP - Each I/O pin (use or unused) is defined . . .
// I2C Pins for RTC - D19 (SCL), D18 (SDA)
#define BUZZER_PIN       16             // sw jumper for buzzer
#define LED_PIN          14             // L1 LED on Logger Shield
// D13 (CLK), D12 (MISO), D11 (MOSI)    SPI Pins for SD card
#define CS_PIN           10             // Logger Shield Chip Select pin for the SD card
//                        D9 & D8          available
//                        D7 D6            serial comm to GPS
//                        D4 D3            available
//                        D2               Interrupt 1 for Geiger 
//                        D1 & D0          interface to pc

// These are DEFAULTS! - only used if menu has not been run
#define DOSE_PARAM        1             // defaults to Dose Mode ON
// #define PRI_RATIO       175             // defaults to SBM-20 ratio
#define PRI_RATIO       122             // defaults to SI-29BG ratio
#define LOGGING_PEROID    1             // defaults a 1 min logging period
// other defines . . .
#define LOW_VCC          4200 //mV      // if Vcc < LOW_VCC give low voltage warning
// RTTY PARAMETERS
#define R_STRENGTH     6
#define R_BAUD         100
#define R_BITS         7
#define R_STOP         2
#define R_WAIT         1000000 / R_BAUD // 50 Baud  = 20.000 microseconds
#define R_FREQ1         433.0
#define R_FREQ2         433.0005

// BEGIN USER PARAMETER AREA . . .
#define DEBUG       true              // if true shows available memory
// END USER PARAMETER AREA

#include <SdFat.h>                      // the complete SD library is too fat (pun intended)  

#include <SPI.h>
#include <RFM22.h>
 
#include <Time.h>                       // time functions
#include <Wire.h>                       // Two Wire interface
#include <Adafruit_BMP085.h>            // BMP085 barometric module
#include <TinyGPS.h>                    // MODIFIED LIB - parses GPS sentences
// MODIFIED LIB - _GPS_NO_STATS uncommented (and resulting compile error fixed) saves 80 bytes
#include <avr/pgmspace.h>               // for using PROGMEM or F("")
#include <SoftwareSerial.h>             // Had too much troubles with HW TX/RX so had to bypass that issue...

////////////////////////////////// globals /////////////////////////////////////

// These hold the local values that have been read from EEPROM
unsigned long LoggingPeriod;            // mS between writes to the card
float uSvRate;                          // holds the rate selected by jumper

boolean SD_OK = true;                   // assume SD is OK until init
boolean lowVcc = false;                 // true when Vcc < LOW_VCC

// variables for counting periods and counts . . .
unsigned long dispPeriodStart, dispPeriod; // for display period

unsigned long logPeriodStart;           // for logging period
unsigned long logCnt, logCPM;           // to count and log CPM

float uSv = 0.0;                        // display CPM converted to VERY APPROXIMATE uSv
float uSvLogged = 0.0;                  // logging CPM converted to VERY APPROXIMATE uSv
float avgCnt;                           // holds the previous moving average count
byte sampleCnt;                         // the number of samples making up the moving average
float temp_uSv = 0.0;                   // for converting CPM to uSv/h for running average
byte altDispCnt = 0;                    // counter to disp running average once / 4 displays
int Vcc_mV;                             // mV of Vcc from last check 

char buffer[12];
char superbuffer[100];
byte day_of_week,DST;                   // if daylight savings time, DST == true
int GMT_Offset;                         // defaults to 13 = GMT+1
unsigned long counter;                  // counter for logcount

// Communication with GPS
SoftwareSerial GPSModul(6, 7);          // 6 is RX, 7 is Tx
TinyGPS gps;                            // GPS conversion library

// Communication with SD card reader
SdFat sd;
SdFile logfile;
SdFile gpsfile;

// BMP085 barometric sensor
Adafruit_BMP085 bmp;                    // barometric sensor

// radio code here
//Setup radio on SPI with NSEL on pin 9
rfm22 radio1(9);


// ---------------- INIT ---------------
void setup(){
  counter=0;
  Serial.begin(9600);                // comspec for GPS is 48,N,8,1
  GPSModul.begin(9600);

  attachInterrupt(0,GetEvent,FALLING);  // Geiger event on D2 triggers interrupt
//  digitalWrite(2, HIGH);              // set pull up on INT0
  pinMode(LED_PIN, OUTPUT);             // setup LED pin
  pinMode(10, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);          // sw control buzzer
  digitalWrite(BUZZER_PIN, HIGH);       // buzzer = on

  Get_Settings();                       // get the settings stored in EEPROM
  LoggingPeriod = 5 * 1000;            // every 5 seconds write entry to Logfile...

  initCard();                           // init the SD card
  initTime();							// read time from SD card 
  initLog();                            // prepare log files
  initGPS();                            // init the GPS to send output only when requested
  setSyncProvider(gpsTimeSync);         // define the function that syncs the time with GPS

//  Serial.println("Init BMP");  

  if (!bmp.begin()) {
    Serial.println(F("No BMP085"));
  }

// Setup Radio
  setupRadio();
  Serial.print("RAM:");
  Serial.println(AvailRam());

  logPeriodStart = 0;     // start logging timer
  logCnt= 0;
}

void Get_Settings(){ // read setting out of EEPROM and set local variables
  uSvRate = PRI_RATIO;
  GMT_Offset = 1; // readParam(ZONE_ADDR);
  logPeriodStart = 0;     // start logging timer
}

// ---------------- MAIN LOOP ---------------

void loop(){
  //uncomment 4 lines below for self check (1/166ms X 60 = 360 CPM
  //dispCnt++;
  //logCnt++;
  //runCnt++;
  //delay(167);                         // 167 mS ~= 6 Hz    
 
    readGPS();  // make sure all serial input is parsed...
  
    if (millis() >= logPeriodStart + LoggingPeriod){ // LOGGING PERIOD
      if (SD_OK) LogCount(logCnt);      // pass in the counts to be logged
      
      logCnt = 0;                       // reset log event counter
      logPeriodStart = millis();        // reset log time
    }
}

void GetEvent(){   // ISR triggered for each new event (count)
  logCnt++;
}

// ---- some util functions  ---

void Blink(byte led, byte times){ // just to flash the LED
  for (byte i=0; i< times; i++){
    digitalWrite(led,HIGH);
    delay (120);
    digitalWrite(led,LOW);
    delay (90);
  }
}


long readVcc() { // SecretVoltmeter from TinkerIt
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

int AvailRam(){ 
  int memSize = 2048;                   // if ATMega328
  byte *buf;
  while ((buf = (byte *) malloc(--memSize)) == NULL);
  free(buf);
  return memSize;
} 


///////////////////////////////// Time Functions Here ///////////////////////////////

void FormatDate(){  // get the time and date from the GPS and format it
  int dispYear;
  int i;
  memset(buffer,0,sizeof(buffer));

  // convert Sun=1 format to Sun=7 format (DST calc is based on this)
  day_of_week = (weekday()==1) ? 7: (weekday() - 1);

  // make date string
  i=day();                              // add DAY to string
  if (i < 10) strcat(buffer,"0");
  AppendToString (i,buffer);  
  strcat(buffer, ".");

  i = month();                          // add MONTH to string  
  if (i < 10) strcat(buffer,"0");
  AppendToString (i,buffer);
  strcat(buffer, ".");

  i = year();                          // add YEAR to string
  AppendToString (i,buffer);                            
}

void FormatTime(){  // get the time and date from the GPS and format it
  int dispYear;
  int i;
  memset(buffer,0,sizeof(buffer));  // initialize the strings

  // convert Sun=1 format to Sun=7 format (DST calc is based on this)
  day_of_week = (weekday()==1) ? 7: (weekday() - 1);

  // (The time lib will deal with AM/PM and 12 hour clock)
  // make time string
  AppendToString (hour(),buffer);         // add 24 hour time to string
  strcat(buffer,":");
  if (minute() < 10) strcat(buffer,"0");
  AppendToString (minute(),buffer);       // add MINUTES to string
  strcat(buffer,":");
  if (second() < 10) strcat(buffer,"0");
  AppendToString (second(),buffer);     // add SECONDS to string
}


void AppendToString (int iValue, char *pString){ // appends a byte to string passed
  char tempStr[6];
  memset(tempStr,'\0',sizeof(tempStr));
  itoa(iValue,tempStr,10);
  strcat(pString,tempStr);
}

void FAppendToString (float fValue, char *pString){ // appends a byte to string passed
  char tempStr[12];
  memset(tempStr,'\0',sizeof(tempStr));

  ftoa(fValue,tempStr);
  strcat(pString,tempStr);
}


void HAppendToString (int iValue, char *pString){ // appends a byte to string passed
  char tempStr[12];
  memset(tempStr,'\0',sizeof(tempStr));
//  if (iValue<0x1000) strcat(pString, "0");
//  if (iValue<0x100) strcat(pString, "0");
  if (iValue<0x10) strcat(pString, "0");
  itoa(iValue, tempStr, 16);
  strcat(pString,tempStr);
}


/////////////////////////////////// GPS Functions ////////////////////////////////
bool readGPS(){  //  request and get a sentence from the GPS
char c;
boolean res;
  res = false;
  while (GPSModul.available()){           // Get sentence from GPS
    c = GPSModul.read();
    if (SD_OK) {
       gpsfile.print(c);
    }   
    if (gps.encode(c)) res = true;
  }
  return res;
}


time_t gpsTimeSync(){  //  returns time if avail from gps, else returns 0
  unsigned long fix_age = 0 ;
  gps.get_datetime(NULL,NULL, &fix_age);

  if(fix_age < 1000) return gpsTimeToArduinoTime();   // only return time if recent fix  
  return 0;
}


time_t gpsTimeToArduinoTime(){  // returns time_t from GPS with GMT_Offset for hours
  tmElements_t tm;
  int year;
  readGPS();                    // make sure we process te most recent data
  gps.crack_datetime(&year, &tm.Month, &tm.Day, &tm.Hour, &tm.Minute, &tm.Second, NULL, NULL);
  tm.Year = year - 1970; 
  time_t time = makeTime(tm);
  if (InDST()) time = time + SECS_PER_HOUR;
  return time + (GMT_Offset * SECS_PER_HOUR);
}


bool InDST(){  // Returns true if in DST - Caution: works for central europe only
  // DST starts the last Sunday in March and ends the last Sunday in Octobre
/*  
  bool res;
  byte DOWinDST, nextSun;
  int Dy, Mn;  

  Dy = tmDay;
  Mn = tmMonth;
  //Dy = 27;  // FOR TESTING
  //Mn = 7;
  res = false;

  // Pre-qualify for in DST in the widest range (any date between  and 23.10) 
  // Earliest date in March is 25th
  // Earliest date in October is 25th
  if ((Mn == 3 && Dy >= 25) || (Mn > 3 && Mn < 10) || (Mn == 10 && Dy <= 23) ){
    DOWinDST = true;                    // assume it's in DST until proven otherwise
    nextSun = Dy + (7 - day_of_week);   // find the date of the last Sunday
  while (nextSun<24) nextSun += 7;
    if (nextSun > 31) nextSun -= 7;     // come back to month
    if (Mn == 3 && (Dy < nextSun)) DOWinDST = false;     // it's before the last Sun in March
    if (Mn == 10 && (Dy >= nextSun)) DOWinDST = false; // it's after the 1ast Sun in Oct.
    if (DOWinDST) res = true;           // DOW is still OK so it's in DST
  }
  return res;                         // Nope, not in DST
*/
  return false;
}

void initGPS(){   // send commands to the GPS for what to output - uses PROGMEM for this
  // could program GPS by sending some commands to ommit cerrtain stats, but why ? :-)

}
////////////////////////////// Logging Functions Here ///////////////////////////////
unsigned int gps_checksum (char * string)
{	
	unsigned int i;
	unsigned int XOR;
	unsigned int c;
	// Calculate checksum ignoring any $'s in the string
	for (XOR = 0, i = 0; i < strlen(string); i++)
	{
		c = (unsigned char)string[i];
		if (c != '$') XOR ^= c;
	}
	return XOR;
}

void LogCount(unsigned long lcnt){
  // SD File format: "Date", "Time", latitude, longitude, speed, CPM, uSv/hr, Vcc, temperature, pressure1, pressure2
  bool newdata = false; //TODO
  int n;
  uint16_t crc;
  float flat, flon, alti;
  float bmp_temp;
  int32_t bmp_pascal;
  uint32_t bmp_raw_pascal;
  unsigned long age;
  time_t nw;
  
  nw = gpsTimeSync();
  if (nw==0) nw = now();

/*  
  Serial.print("Logcount: ");
  Serial.println(lcnt);
*/
  counter++;
  bmp_temp = bmp.readTemperature();
  bmp_pascal = bmp.readPressure();
  bmp_raw_pascal = bmp.readRawPressure();
/*
  Serial.print("T="); Serial.println(bmp_temp);  
  Serial.print("P="); Serial.println(bmp_pascal);
*/
  Vcc_mV = readVcc();                   // check voltage
  if (Vcc_mV <= LOW_VCC) lowVcc = true; // check if Vcc is low 
  else lowVcc = false;
  readGPS();                              // get a reading if any present
  newdata = true; // FORCING THIS FOR NOW

  gps.f_get_position(&flat, &flon, &age); // get the current GPS position
  alti = gps.altitude() / 100.0;           // calc in m
 // Log counts . . .
  logCPM = float(lcnt) / (float(LoggingPeriod) / 60000);
  uSvLogged = logCPM / uSvRate;         // make uSV conversion

// ID,NR,TIME,LAT,LON,KMPH,ALT,FIX,SATS,CPM,uSv,Vcc,TEMP,PR1,PR2,CRC
   superbuffer[0]='\x0';
   strcat(superbuffer, "$$PYSY3,""");
   FormatTime();
   strcat(superbuffer, buffer);
   strcat(superbuffer, """,""");
   FormatTime();
   strcat(superbuffer, buffer);
   strcat(superbuffer, """,");
   FAppendToString(flat, superbuffer);
   strcat(superbuffer, ",");
   FAppendToString(flon, superbuffer);
   strcat(superbuffer, ",");
   FAppendToString(alti, superbuffer);
   strcat(superbuffer, ",");
/*    i=gps.fixtype();
   AppendToString(i, superbuffer);
   strcat(superbuffer, ",");
*/   
   n=gps.satellites();
   AppendToString(n, superbuffer);
   strcat(superbuffer, ",");

   FAppendToString(logCPM, superbuffer);
   strcat(superbuffer, ",");
   FAppendToString(uSvLogged, superbuffer);
   strcat(superbuffer, ",");
   FAppendToString(Vcc_mV, superbuffer);
   strcat(superbuffer, ",");
   FAppendToString(bmp_temp, superbuffer);
   strcat(superbuffer, ",");
   FAppendToString(bmp_pascal, superbuffer);
   strcat(superbuffer, ",");
   FAppendToString(bmp_raw_pascal, superbuffer);

   crc = gps_checksum(superbuffer);
//   n = sprintf (superbuffer, "%s*%02X\n", superbuffer, gps_checksum(superbuffer));     
   strcat(superbuffer, "*");
   HAppendToString(crc, superbuffer);
   strcat(superbuffer, "\x0d");
   Serial.println(superbuffer);
//   

  logfile.println(superbuffer);
  // really need to tell if the GPS has a fix
  if (newdata = true) Blink(LED_PIN,1); // show it's receiving
  logfile.sync();                       // force update of the files
  gpsfile.sync();
  radio1.write(0x6d, R_STRENGTH);
  radio1.write(0x07, 0x08); // Radio on
  rtty_txstring(superbuffer);
  radio1.write(0x07, 0x01); // Radio off
}

void initCard(){   // initialize the SD card
  SD_OK = false;                        // don't try to write to the SD card
  pinMode(10, OUTPUT);                  // must set DEFAULT CS pin output, even if not used

  if (!sd.begin(10, SPI_HALF_SPEED))
 {  
    sd.initErrorHalt();
    error("Card");
  }
  SD_OK = true;
//  SdFile::dateTimeCallback(SDdateTime); 
  SdFile::dateTimeCallback(SDDateTime);
}

void initLog(){
  char filename[]="py000.csv";
  if ( gpsfile.open("pysy.log", O_WRONLY | O_CREAT) ) {
    gpsfile.println("PYSY-Startup");
    FormatDate();
    gpsfile.println(buffer);
    FormatTime();
    gpsfile.println(buffer);
    gpsfile.print("RAM:");
    gpsfile.println(AvailRam());
    gpsfile.sync();
  }
  else  
  {
    sd.errorHalt("pysy.log not writeable");
  }  
  
  for (uint8_t i = 0; i < 250; i++) {
    filename[2] = i/100 + '0';
    filename[3] = (i%100)/10 + '0';
    filename[4] = i%10 + '0';
/*    
    Serial.print("Trying: ");
    Serial.println(filename);
*/
    if (! sd.exists(filename)) {
        break;  // leave the loop!
    }  
  }
//  Serial.print("Final test:"); Serial.println(filename);
  if (logfile.open(filename, O_WRONLY | O_CREAT) ) {
//    Serial.println(F("SD OK, now write..."));
//    logfile.println(F("ID,NR,DATE,TIME,LAT,LON,KMPH,ALT,FIX,SATS,CPM,uSv,Vcc,TEMP,PR1,PR2,CRC"));
  }
  else
  {
     sd.errorHalt("opening logfile for write failed");
  }  

}

void error(char *str){
  Serial.println("CARD!");
  Serial.println(str);                       // display this error or status
  digitalWrite(LED_PIN, HIGH);          // red LED indicates error
}

/* Reads default time before any fix from GPS forces the right one, 
   tries to read time.txt from SD-card 
   Attention: uses logfile variable ! */
void initTime()
{
  char ch;
  int tpos[6];
  int valu, ct, rd;
  tpos[6]=2012; // YEAR
  tpos[5]=11;   // MONTH
  tpos[4]=1;   // DAY
  tpos[3]=0;    // SEC
  tpos[2]=0;    // MIN
  tpos[1]=14;   // HOUR
/*  
  if (SD_OK)
  {
    ct = 1;
    valu = 0;
    if (logfile.open("time.txt", O_READ))
    {
       while ((rd = logfile.read()) >= 0){
         ch = (char)rd;
         switch (ch) {
         case '\n':
           if (ct<=6) tpos[ct] = valu;
           ct = ct + 1;
           valu = 0;
           break;
         case '0': valu = valu * 10 + 0; break;
         case '1': valu = valu * 10 + 1; break;
         case '2': valu = valu * 10 + 2; break;
         case '3': valu = valu * 10 + 3; break;
         case '4': valu = valu * 10 + 4; break;
         case '5': valu = valu * 10 + 5; break;
         case '6': valu = valu * 10 + 6; break;
         case '7': valu = valu * 10 + 7; break;
         case '8': valu = valu * 10 + 8; break;
         case '9': valu = valu * 10 + 9; break;
         default:
           break;  
         }  
       } 
      logfile.close(); 
    }
  }
*/
/*
  Serial.print("Set Time:");
  for (ct=1;ct<4;ct++){
     Serial.print(tpos[ct]); 
     Serial.print(":");
  }   
  Serial.print(" ");
  for (ct=4;ct<7;ct++){
     Serial.print(tpos[ct]); 
     Serial.print(".");
  }  
  Serial.println();
*/
  setTime(tpos[1], tpos[2], tpos[3], tpos[4], tpos[5], tpos[6]);
}

// call back for file timestamps
void SDDateTime(uint16_t* date, uint16_t* time) {
  time_t nw;
  nw = gpsTimeSync();
  if (nw==0) nw = now();
/*  
  Serial.print("SDDate:" );
  Serial.print(day(nw));  
  Serial.print(".");
  Serial.print(month(nw));  
  Serial.print(".");
  Serial.println(year(nw));
*/
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(nw), month(nw), day(nw));

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(nw), minute(nw), second(nw));
}

// RADIO CODE WITH RFM22
void setupRadio(){
 
//  digitalWrite(5, LOW); // Radio anschalten falls SDN mit Pin 5 verbunden
 
//  delay(1000);
 
  rfm22::initSPI();
 
  radio1.init();
 
  radio1.write(0x71, 0x00); // unmodulated carrier
 
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
 
  radio1.setFrequency(R_FREQ1);
 
  //Quick test
  radio1.write(0x6d, R_STRENGTH); // set output power: RF22_REG_6D_TX_POWER: 00 low , 07 high
  radio1.write(0x07, 0x08); // turn tx on
  delay(1000);
  radio1.write(0x07, 0x01); // turn tx off
  /*
 #define RF22_TXPOW_1DBM                         0x00
#define RF22_TXPOW_2DBM                         0x01
#define RF22_TXPOW_5DBM                         0x02
#define RF22_TXPOW_8DBM                         0x03
#define RF22_TXPOW_11DBM                        0x04
#define RF22_TXPOW_14DBM                        0x05
#define RF22_TXPOW_17DBM                        0x06
#define RF22_TXPOW_20DBM                        0x07
*/
 
}

// RTTY Functions - from RJHARRISON's AVR Code
void rtty_txstring (char * string)
{
 
	/* Simple function to sent a char at a time to 
	** rtty_txbyte function. 
	** NB Each char is one byte (8 Bits)
	*/
	char c;
	c = *string++;
	while ( c != '\0')
	{
		rtty_txbyte (c);
		c = *string++;
	}
        Serial.println();
}
 
void rtty_txbyte (char c)
{
	/* Simple function to sent each bit of a char to 
	** rtty_txbit function. 
	** NB The bits are sent Least Significant Bit first
	**
	** All chars should be preceded with a 0 and 
	** proceded with a 1. 0 = Start bit; 1 = Stop bit
	**
	** ASCII_BIT = 7 or 8 for ASCII-7 / ASCII-8
	*/
	int i;
        Serial.print(c);
        
	rtty_txbit (0); // Start bit
	// Send bits for for char LSB first	
	for (i=0;i<R_BITS;i++)
	{
		if (c & 1) rtty_txbit(1); 
			else rtty_txbit(0);	
		c = c >> 1;
	}
        for (i=0; i<R_STOP; i++) 
        {
           rtty_txbit (1); // Stop bit
        }   
}
 
void rtty_txbit (int bit)
{
  if (bit)
  {
  // high
    radio1.setFrequency(R_FREQ1);
  }
  else
  {
	// low
    radio1.setFrequency(R_FREQ2);
  }
  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
  delayMicroseconds(10150); // For some reason you can't do 20150 it just doesn't work.
 
}

void ftoa(float Value, char* Buffer)
 {
     union
     {
         float f;
     
         struct
         {
             unsigned int    mantissa_lo : 16;
             unsigned int    mantissa_hi : 7;    
             unsigned int     exponent : 8;
             unsigned int     sign : 1;
         };
     } helper;
     
     unsigned long mantissa;
     signed char exponent;
     unsigned int int_part;
     char frac_part[3];
     int i, count = 0;
     
     helper.f = Value;
     //mantissa is LS 23 bits
     mantissa = helper.mantissa_lo;
     mantissa += ((unsigned long) helper.mantissa_hi << 16);
     //add the 24th bit to get 1.mmmm^eeee format
     mantissa += 0x00800000;
     //exponent is biased by 127
     exponent = (signed char) helper.exponent - 127;
     
     //too big to shove into 8 chars
     if (exponent > 18)
     {
         Buffer[0] = 'I';
         Buffer[1] = 'n';
         Buffer[2] = 'f';
         Buffer[3] = '\0';
         return;
     }
     
     //too small to resolve (resolution of 1/8)
     if (exponent < -3)
     {
         Buffer[0] = '0';
         Buffer[1] = '\0';
         return;
     }
     
     count = 0;
     
     //add negative sign (if applicable)
     if (helper.sign)
     {
         Buffer[0] = '-';
         count++;
     }
     
     //get the integer part
     int_part = mantissa >> (23 - exponent);    
     //convert to string
     itoa(int_part, &Buffer[count],10);
     
     //find the end of the integer
     for (i = 0; i < 8; i++)
         if (Buffer[i] == '\0')
         {
             count = i;
             break;
         }        
 
     //not enough room in the buffer for the frac part    
     if (count > 5)
         return;
     
     //add the decimal point    
     Buffer[count++] = '.';
     
     //use switch to resolve the fractional part
     switch (0x7 & (mantissa  >> (20 - exponent)))
     {
         case 0:
             frac_part[0] = '0';
             frac_part[1] = '0';
             frac_part[2] = '0';
             break;
         case 1:
             frac_part[0] = '1';
             frac_part[1] = '2';
             frac_part[2] = '5';            
             break;
         case 2:
             frac_part[0] = '2';
             frac_part[1] = '5';
             frac_part[2] = '0';            
             break;
         case 3:
             frac_part[0] = '3';
             frac_part[1] = '7';
             frac_part[2] = '5';            
             break;
         case 4:
             frac_part[0] = '5';
             frac_part[1] = '0';
             frac_part[2] = '0';            
             break;
         case 5:
             frac_part[0] = '6';
             frac_part[1] = '2';
             frac_part[2] = '5';            
             break;
         case 6:
             frac_part[0] = '7';
             frac_part[1] = '5';
             frac_part[2] = '0';            
             break;
         case 7:
             frac_part[0] = '8';
             frac_part[1] = '7';
             frac_part[2] = '5';                    
             break;
     }
     
     //add the fractional part to the output string
     for (i = 0; i < 3; i++)
         if (count < 7)
             Buffer[count++] = frac_part[i];
     
     //make sure the output is terminated
     Buffer[count] = '\0';
 }
 

