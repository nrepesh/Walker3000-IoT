/*****************************************************************
Final Project Walker 3000
OGNT- old generation new technology
stepcounter + 2
date time temp
audio assistance
-location
-voice control
-help emergency assistance
*****************************************************************/
#include <Base64.h>
#include <MPU9250.h>
#include <U8g2lib.h>
#include <math.h>
#include <Wifi_S08_v2.h>
#include <SPI.h>
#include <Wire.h>


#define SPI_CLK 14
#define WIFI_SERIAL Serial1
#define GPSSerial Serial3
#define SERIAL_YES true
MPU9250 imu;
ESP8266 wifi = ESP8266(0,true);

#define DELAY 1000
#define SAMPLE_FREQ 8000                           // Hz, telephone sample rate
#define SAMPLE_DURATION 3                          // sec
#define NUM_SAMPLES SAMPLE_FREQ*SAMPLE_DURATION    // number of of samples
#define ENC_LEN (NUM_SAMPLES + 2 - ((NUM_SAMPLES + 2) % 3)) / 3 * 4  // Encoded length of clip
#define API_KEY AIzaSyAqp1AL6aFSgC8SDc9DTW83lprCNuABRxM

// display
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI oled(U8G2_R2, 10, 15, 16);
// oled.setFont(u8g2_font_ncenB08_tr); 

#define ARRAY_SIZE 10               // number of accel elements to hold

const int UPDATE_INTERVAL = 5000;    // how often to update display (ms)
const int ORDER = 4;                    // order of moving average (samples)
const int STEP_COUNTING = 1;        // are we counting steps?
const int DISPLAYING = 1;           // are we sending to display?

int step_count = 0;                   // # of steps
elapsedMillis time_since_update = 0;  // time since last screen update
elapsedMillis wait_time = 0;                    // time since last step
float accel_data[ARRAY_SIZE] = {0};   // initialize array holding accel data
float mag_accel;                      // acceleration magnitude


/* CONSTANTS */
const char PREFIX[] = "{\"config\":{\"encoding\":\"MULAW\",\"sampleRate\":8000},\"audio\": {\"content\":\"";
const char SUFFIX[] = "\"}}";
const int PREFIX_SIZE = sizeof(PREFIX);
const int SUFFIX_SIZE = sizeof(SUFFIX);
const int BIT_DEPTH = 16;
const int LED_PIN = 13;
const int AUDIO_IN = A3;
const int BUTTON_PIN = 9;

/* Global variables*/
char speech_data[PREFIX_SIZE + ENC_LEN + SUFFIX_SIZE];


int button_state = 0;
elapsedMicros time_since_sample;      // microseconds

String data = "";

const int MAXLENGTH = 140;

//WIFI global constants and variables
const int WIFI_CONTROL_PIN = 2;
String wifis = "";                // holds list of wifis
String MAC = "";                  // MAC address
String get_response ="";          // generic string to hold responses
bool wifi_good = false;           // connected to AP
String SSID = "Sanjeeb";     // SSID and password
String password = "orange123";

class GPSParser
{
  public:
  String valid;
  String lat, NS, lon, EW;
  

  GPSParser(){
    valid = "A";   //Should contain "A" or "V"
    lat = "";
    NS = "";
    lon = "";
    EW = "";
   }

  //Finds commas in "s" and fills in pre-initialized array commaLoc with 
  //the indices of those commas, up to a total of "maxCommas" commas.
  //If there are fewer that "maxCommas" commas, this function pads the
  //remainder of commaLoc (up to index "maxCommas") with the value -1
  void findCommas(String s, int commaLoc[], int maxCommas) {
    commaLoc[0] = s.indexOf(",");
    for (int i=1; i<maxCommas; i++) {
      commaLoc[i] = s.indexOf(",", commaLoc[i-1]+1);
    }
  }

  void extractData(String d)
  {
    int commaLoc[12];
    findCommas(d, commaLoc, 12);
    valid = d.substring(commaLoc[1]+1, commaLoc[2]);   //Should contain "A" or "V"
    lat = d.substring(commaLoc[2]+1, commaLoc[3]);
    NS = d.substring(commaLoc[3]+1, commaLoc[4]);
    lon = d.substring(commaLoc[4]+1, commaLoc[5]);
    EW = d.substring(commaLoc[5]+1, commaLoc[6]);
    }
};


//Create instance of GPSParser class
GPSParser gps_data;


void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  SPI.setSCK(SPI_CLK);   // move the SPI SCK pin from default of 13  
  oled.begin();
  GPSSerial.begin(9600);

  byte c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.println("MPU9250 is online...");
  
  // Calibrate gyro and accelerometers, load biases in bias registers
  imu.initMPU9250();
  imu.MPU9250SelfTest(imu.selfTest);
  imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);
  imu.initAK8963(imu.factoryMagCalibration);
  imu.getAres();
  imu.getGres();
  imu.getMres();


button_state = digitalRead(BUTTON_PIN);
  if(!button_state) {
  pinMode(LED_PIN, OUTPUT);            // Set up output LED
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  analogReadResolution(BIT_DEPTH);  // Set ADC bit depth
  wifi.begin();
  wifi.connectWifi("Sanjeeb", "orange123");
  memcpy(speech_data, PREFIX, PREFIX_SIZE);   // put PREFIX in front of speech data
  memcpy(speech_data + PREFIX_SIZE - 1 + ENC_LEN, SUFFIX, SUFFIX_SIZE); // put SUFFIX at end
  }else{
  
  //wifi serial setup:
  WIFI_SERIAL.begin(115200);
  WIFI_SERIAL.setTimeout(4000);
  pinMode(WIFI_CONTROL_PIN,OUTPUT);
  wifiEnable(true);
  
  if (check()){     //if ESP8266 is present
    resetWifi();    //reset
    MAC = getMAC(2000);
    emptyRx();
  }
  }    
}


void loop() 
{
  // read data from the GPS in the 'main loop'
  while (GPSSerial.available())  
  {
    char c = GPSSerial.read(); 
    data = data + c; 
    if (c == '\n'){
      if ((data.indexOf("GPRMC") >= 0) && (data.indexOf("GPVTG") >= 0) && data.indexOf("GPVTG") > data.indexOf("GPRMC")) {
        data =  data.substring(data.indexOf("GPRMC"), data.indexOf("$GPVTG"));
        gps_data.extractData(data);
        Serial.println(data);
        data = "";
      }
//      else {
//        data = "";
//      }
    }
  }
    if (data.length() > MAXLENGTH){
      data = "";
    }

  button_state = digitalRead(BUTTON_PIN);
  if(!button_state) {
    delay(200);
    digitalWrite(LED_PIN, HIGH);       // ON means we are recording
    Serial.println("listening...");
    record_audio();
    digitalWrite(LED_PIN, LOW);
    Serial.println("sending...");
    send_from_teensy();
    update_displays();
  } 
  //step counter
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it will store the data
  // in imu.accelCount[] imu.ax, imu.ay, and imu.az variables with the most current data.
  imu.readAccelData(imu.accelCount);
  
  
  // calculate acceleration in g's and store in ax, ay, and az
  imu.ax = (float)imu.accelCount[0]*imu.aRes;
  imu.ay = (float)imu.accelCount[1]*imu.aRes;
  imu.az = (float)imu.accelCount[2]*imu.aRes;

  // calculate the acceleration magnitude
  mag_accel = sqrt(pow(imu.ax,2)+pow(imu.ay,2)+pow(imu.az,2));

   if (STEP_COUNTING) { 
    // shift new data into array
    update_array(accel_data, mag_accel);

    // smooth the data with moving-average filter
    float smoothed_data[ARRAY_SIZE] = {0};  
    smooth_data(accel_data, smoothed_data, ORDER);

    // look for peak, if peak is above threshold AND enough time has passed 
    // since last step, then this is a new step
    float peak = find_peak(smoothed_data);    //EDIT THIS FUNCTION BELOW

    // MORE CODE HERE TO UPDATE THE VARIABLE step_count
    if (wait_time>500 && peak>1.5){
    step_count=step_count+2;
    wait_time=0;
    }
    
  
  // update the display periodically
  if (DISPLAYING) {
    if (time_since_update >= UPDATE_INTERVAL) {
      update_displays();      
      time_since_update = 0;
    }
  }
   }
  
 
  update_display();
// weather 
  if (wifi_good != true){   //if we're not connected to a network

    if (check()){       //if ESP8266 present
      listWifis();        
      if (SERIAL_YES) Serial.println(wifis);

      bool good = connectWifi(SSID,password); //connect to network

     if (good){
        wifi_good = true;   // connected to a network
      }else{
        wifi_good=false;
      }
         
   }
    

  if (wifi_good){   //if we're connected to a network
  
    // web server parameters
    String domain = "iesc-s2.mit.edu";
    int port = 80;
    String path = "/6S08dev/weather/";

    // send a GET command, will return weather info
    String send_comm = 
    "GET "+ path + " HTTP/1.1\r\n" +
    "Host: " + domain + "\r\n\r\n";

    get_response = httpComm(domain, port, path, send_comm);
    if (SERIAL_YES) Serial.println(get_response);
  }
      
     } 

}

void record_audio() {
  int sample_num = 0;    // counter for samples
  int enc_index = PREFIX_SIZE - 1;  // index counter for encoded samples
  float time_between_samples = 1000000 / SAMPLE_FREQ;
  int value = 0;
  int8_t raw_samples[3];   // 8-bit raw sample data array
  char enc_samples[4];     // encoded sample data array

  while (sample_num < NUM_SAMPLES) { //read in NUM_SAMPLES worth of audio data
    time_since_sample = 0;
    value = analogRead(AUDIO_IN);
    raw_samples[sample_num % 3] = mulaw_encode(value - 24427);
    sample_num++;
    if (sample_num % 3 == 0) {
      base64_encode(enc_samples, (char *) raw_samples, 3);
      for (int i = 0; i < 4; i++) {
        speech_data[enc_index + i] = enc_samples[i];
      }
      enc_index += 4;
    }

    // wait till next time to read
    while (time_since_sample <= time_between_samples) delayMicroseconds(10);
  }
}

void send_from_teensy() {
  Serial.println("sending audio data from Teensy");
  wifi.sendBigRequest("speech.googleapis.com", 443, "/v1beta1/speech:syncrecognize?key=AIzaSyAqp1AL6aFSgC8SDc9DTW83lprCNuABRxM", speech_data);
}


/* This code was obtained from
  http://dystopiancode.blogspot.com/2012/02/pcm-law-and-u-law-companding-algorithms.html
*/
int8_t mulaw_encode(int16_t number)
{
  const uint16_t MULAW_MAX = 0x1FFF;
  const uint16_t MULAW_BIAS = 33;
  uint16_t mask = 0x1000;
  uint8_t sign = 0;
  uint8_t position = 12;
  uint8_t lsb = 0;
  if (number < 0)
  {
    number = -number;
    sign = 0x80;
  }
  number += MULAW_BIAS;
  if (number > MULAW_MAX)
  {
    number = MULAW_MAX;
  }
  for (; ((number & mask) != mask && position >= 5); mask >>= 1, position--)
    ;
  lsb = (number >> (position - 4)) & 0x0f;
  return (~(sign | ((position - 5) << 4) | lsb));
}

/* UTILITY FUNCTIONS for step counter*/

// shift array once to left, add in new datapoint
void update_array(float *ar, float newData)
{
  int s = ARRAY_SIZE;
  for (int i=0; i<=s-2; i++) {
    ar[i] = ar[i+1];
  }
  ar[s-1] = newData;
}


// m point moving average filter of array ain
void smooth_data(float *ain, float *aout, int m)
{
  int s = ARRAY_SIZE;
  for (int n = 0; n < s ; n++) {
    int kmin = n>(m-1) ? n - m + 1: 0;
    aout[n] = 0;
  
    for (int k = kmin; k <= n; k++) {
      int d = m > n+1 ? n+1 : m;
      aout[n] += ain[k] / d;
    }
  }
}


// find peak in the array
float find_peak(float *ar)
{
  int s = ARRAY_SIZE;   // s is length of vector ar
  float p = 0;
  for (int i = 1; i<s-1; i++) {
    if ((ar[i] >= ar[i-1]) && (ar[i] >= ar[i+1])) {
      p = ar[i];
    }
  }
  return p;
}

void update_displays()
{
  if (wifi.hasResponse()) {
    String r = wifi.getResponse();
    Serial.print("Got Response! ");
    Serial.println(r);
     // parse get_response
    // WRITE AND UPDATE THIS CODE
    int text_start = r.indexOf(": ") + 3;
    int text_end = r.indexOf("\",");
    String text = r.substring(text_start, text_end);

     // update display
       Serial.println(text);
        String shape = r.substring(text_start, text_end).toLowerCase();
        int triangle = shape.indexOf("triangle");
    int snowman = shape.indexOf("build a snowman");    
    int help = shape.indexOf("help");
    int location = shape.indexOf("where am i");
    

    if (help >= 0) {
      oled.clearBuffer();
      oled.setFont(u8g2_font_ncenB08_tr); 
      oled.drawStr(0 ,20, "CALLING 911..."); 
      oled.sendBuffer();
      delay(5000);
    }else if (location >= 0) {
      oled.clearBuffer();
      oled.setFont(u8g2_font_ncenB08_tr); 
      oled.drawStr(0 ,20, "GPS LOCATION");
      oled.setCursor(0,45);
      oled.print("Lat: " + gps_data.lat.substring(0,2) + "' " + gps_data.lat.substring(2) + " " + gps_data.NS);
      oled.setCursor(0,60);
      oled.print("Lon: " + gps_data.lon.substring(0,3) + "' " + gps_data.lon.substring(3) + " " + gps_data.EW);
      oled.sendBuffer();
      delay(5000);
    }else if (triangle >= 0) {
      oled.clearBuffer();
      oled.setFont(u8g2_font_ncenB08_tr); 
      oled.drawStr(60 ,60, "OK..."); 
      oled.drawTriangle(15,30,20,15, 30,30);
      oled.sendBuffer();
      delay(5000);
    }else if (snowman >= 0) {
      oled.clearBuffer();    //clear the screen contents 
      oled.setFont(u8g2_font_ncenB08_tr); 
      oled.drawStr(0, 35, "C'mon let's go out and"); 
      oled.drawStr(55, 45, "play!!!"); 
      oled.setFont(u8g2_font_unifont_t_symbols);
      oled.drawGlyph(5, 20, 0x2603);  /* dec 9731/hex 2603 Snowman */
      oled.sendBuffer();
      delay(5000);
    }else{
       
       oled.clearBuffer();       
       oled.setFont(u8g2_font_ncenB08_tr); 
          // MORE CODE HERE
       oled.drawStr(0 ,10, "Hi! You Just Said..."); 
       oled.setCursor(20, 35);
       oled.print(text);
       oled.setCursor(0, 43);
       oled.print("~~~~~~~~~~~~~~~~~~~~~~~~~~");
       oled.sendBuffer();     // update the screen
    delay(2000);
      
    }
    }else{ 
   oled.clearBuffer();
 oled.setFont(u8g2_font_ncenB08_tr); 
 oled.setCursor(0,20);
  oled.print("STEPS YOU WALKED");
  oled.setCursor(0,45);
  oled.print("STEPS- " + String(step_count));
  oled.setCursor(0,60);
  oled.print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  oled.sendBuffer(); 
  delay(1000);
    }
}

void update_display()
{
   oled.clearBuffer();
 oled.setFont(u8g2_font_ncenB08_tr); 
  oled.setCursor(0,45);
  oled.print("STEPS- " + String(step_count));
  
    int date_start = get_response.indexOf("~D") + 3;
    int date_end = get_response.indexOf(" ",date_start+1);
    int time_end = get_response.indexOf("~L",date_end+1);
    int temp_start = get_response.indexOf("~T") + 3;
    int temp_end = get_response.indexOf("</body>",temp_start+1);

    String date = get_response.substring(date_start, date_end);
    String time = get_response.substring(date_end, time_end);
    String tempC = get_response.substring(temp_start, temp_end);
    float tempF = 0;
   
   oled.setCursor(0, 15);
   oled.print("DATE- ");
   oled.setCursor(38, 15);
   oled.print(date);

   oled.setCursor(0, 30);
   oled.print("TIME- ");
   oled.setCursor(35, 30);
   oled.print(time);


   oled.setCursor(0, 60);
   oled.print("TEMP- ");
   oled.setCursor(40, 60);
   oled.print(tempC);
   oled.sendBuffer();  
  
}


//////
//////
///Wifi Functions:
void wifiEnable(bool yn){
  digitalWrite(WIFI_CONTROL_PIN,LOW);
  if (yn){
    delay(250);
    digitalWrite(WIFI_CONTROL_PIN,HIGH);
    delay(1000);
  } 
}


bool check() {
    emptyRx();
    WIFI_SERIAL.println("AT");
    if (SERIAL_YES) Serial.println("checking...");
    boolean ok = false;
    if (WIFI_SERIAL.find("OK")) {
        if (SERIAL_YES) Serial.println("ESP8266 present");
        ok = true;
    }
    return ok;
}


void resetWifi() {
  // set station mode
    WIFI_SERIAL.println("AT+CWMODE=1");
    delay(500);//give some breathing room
    WIFI_SERIAL.println("AT+RST"); //reset required to take effect
    delay(500);
    if (WIFI_SERIAL.find("ready")){
      if (SERIAL_YES) Serial.println("ESP8266 restarted and ready");
    }
    printWifiResponse();
}


bool connectWifi(String ssid, String password) {
    emptyRx();
    String cmd = "AT+CWJAP=\"" + ssid + "\",\"" + password + "\"";
    WIFI_SERIAL.println(cmd);
    unsigned long start = millis();
    String response="";
    
    while (millis() - start <9000){ //probably can rewrite this thing if needed.
      if (WIFI_SERIAL.available()){
        char c = WIFI_SERIAL.read();
        if (SERIAL_YES) Serial.print(c);
        response = String(response+c);
      }
      if (response.indexOf("OK") != -1){
        break;
      }
    }
    if(response.indexOf("OK") !=-1) {
        WIFI_SERIAL.println("AT+CIFSR");
        String resp2 = "";
        start = millis();
        while(millis()-start < 7000){
          if(WIFI_SERIAL.available()){
            char c = WIFI_SERIAL.read();
            //Serial.print(c);
            resp2 = String(resp2+c);
          }
        }
        if (SERIAL_YES){
          Serial.println("Device IP Info: ");
          Serial.println(resp2);
          Serial.println("Connected!");
          return true;
        }
    }
    else {
        if (SERIAL_YES) Serial.println("Cannot connect to wifi");
        return false;
    }
}


void listWifis(){
  wifis="";
  if (SERIAL_YES) Serial.println("Checking for Wifis!!!");
  emptyRx();
  WIFI_SERIAL.println("AT+CWLAP");
  unsigned long start = millis();
  while (millis() - start <5000){
    if (WIFI_SERIAL.available()){
      char c = WIFI_SERIAL.read();
      wifis = String(wifis+c);
    }
  }
}


void printWifiResponse(){
  while (WIFI_SERIAL.available()>0){
    char cByte = WIFI_SERIAL.read();
    Serial.write(cByte);
  }
}


// Send an http command
String httpComm(String domain, int port, String path, String comm) {  

  String response;  
  emptyRx();          // empty buffer
  if (setMux(0)) {    // set mux
    emptyRx();
    
    if (startComm(domain, port)) {  // set up tcp
      emptyRx();
      
      
      if (sendComm(comm, comm.length())) {  //send command
        response = receiveData(5000);       //receive response
      } else {
        Serial.println("Send failed");
      } 
    } else {
      Serial.println("Unable to start connection");
    }
  } else {
    Serial.println("MUX command failed");
  }
  WIFI_SERIAL.println("AT+CIPCLOSE");       //close tcp connection
  return response;
}



// Empty ESP8266 buffer
void emptyRx() {
    while(WIFI_SERIAL.available() > 0) {
        WIFI_SERIAL.read();
    }
}


// Read data from the wifi and test it for any of three target strings
bool readTest(String target1, String target2, String target3, uint32_t timeout) {

    String data_tmp;
    data_tmp = readString(target1, target2, target3, timeout);
    if (data_tmp.indexOf(target1) != -1) {
        return true;
    } else if (data_tmp.indexOf(target2) != -1) {
        return true;
    } else if (data_tmp.indexOf(target3) != -1) {
        return true;
    } else {
    return false;
    }
}

// Read data from the wifi and return that data once any of three target 
// strings are encountered
String readString(String target1, String target2, String target3, uint32_t timeout)
{
    String data;
    char a;
    unsigned long start = millis();
    while (millis() - start < timeout) {
        while(WIFI_SERIAL.available() > 0) {
            a = WIFI_SERIAL.read();
      if(a == '\0') continue;
            data += a;
        }
        if (data.indexOf(target1) != -1) {
            break;
        } else if (data.indexOf(target2) != -1) {
            break;
        } else if (data.indexOf(target3) != -1) {
            break;
        }
    }
    return data;
}


// Set the multiplexing
bool setMux(int m) {
  
  String data;
  WIFI_SERIAL.print("AT+CIPMUX=");  
  WIFI_SERIAL.println(m);  
  data = readString("OK","ERROR","XX",5000);
   
  if (data.indexOf("OK") != -1) {
        return true;
  }
  return false;
}


// Set up TCP connection
bool startComm(String domain, int port) {

  String data;
  String start_comm = "AT+CIPSTART=\"TCP\",\"" + domain + "\"," + String(port);     //UPDATE THIS LINE
  WIFI_SERIAL.println(start_comm);
  if (SERIAL_YES) Serial.println(start_comm);

  data = readString("OK", "ERROR", "ALREADY CONNECT", 10000);
    if (data.indexOf("OK") != -1 || data.indexOf("ALREADY CONNECT") != -1) {
        return true;
    }
  return false;
}


// Send a GET or POST command
bool sendComm(String buffer, int len) {
    WIFI_SERIAL.print("AT+CIPSEND=");   //send length command
    WIFI_SERIAL.println(len);
    if (readTest(">", "XX", "XX", 5000)) {    //if we get '>', send rest
        emptyRx();
        for (uint32_t i = 0; i < len; i++) {
            WIFI_SERIAL.write(buffer[i]);
        }
        return readTest("SEND OK", "XX", "XX", 10000);
    }
    return false;
}


// Read data from wifi and place into string
String receiveData(uint32_t timeout) {
  String response;
  unsigned long start = millis();
  while (millis() - start <timeout){
    if (WIFI_SERIAL.available()>0){
      char c = WIFI_SERIAL.read();
      Serial.print(c);
      response=String(response+c);
    }
  }
  return response;
}


String getMAC(uint32_t timeout) {
  String response;
  WIFI_SERIAL.println("AT+CIPSTAMAC?");   //send MAC query
  unsigned long start = millis();
  
  while (millis() - start <timeout){
    if (WIFI_SERIAL.available()>0){
      char c = WIFI_SERIAL.read();
      Serial.print(c);
      response=String(response+c);
    }
  }
  int stringStart = response.indexOf(":") + 2;
  return response.substring(stringStart,stringStart+17);
}
