//uint8_t IRpin = 2;
// Digital pin #2 is the same as Pin D2 see
// http://arduino.cc/en/Hacking/PinMapping168 for the 'raw' pin mapping


//Fona setup
#include "Adafruit_FONA.h"
#include <EEPROM.h>
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
// this is a large buffer for replies
char replybuffer[255];
// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
// Hardware serial is also possible!
//  HardwareSerial *fonaSerial = &Serial1;
// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
//String yourThing = "cfaairquality";
//String yourThing2 ="sensordata";
//String HttpResponseData ;

//DHT11 Setup
#include <Adafruit_Sensor.h>
#include "DHT.h"
#define DHTPIN 5
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);
// #include <dht.h>
// dht DHT;
// #define DHT11_PIN 5


//Patricle Matter Setup

//CO setup


//ASSIGN PIN TO TRANSDUCERS


//ASSIGN OTHER VARIBALES.
unsigned long time_intval_btw_data__cap_ms = 900000; // 15mins = 900000ms
unsigned long start_cap_time_ms = 0; //  secs
unsigned long end_cap_time_ms = 0; //  secs
int next_state =34 ; // keep the state of things
// help to hold real-time average values of pollutant data captured
        float real_time_average_pm10=0;
        float real_time_average_pm25=0;
        float real_time_average_co=0;
        float real_time_average_tem=0;
        float real_time_average_hum=0;


//constant Web data for setting
String sn ="lembor";//sensor name
String sc = "";//sensor city
int si= 1; //sensor installed.
String se=""; //in care of
String so= "";//organisation in care of
String sp= "";//phone no
String va=""; //pollutant variable
String st=""; //country
//float l= //longitude
//float a= //latitude
//float h= //height
//int v= //data interval
String r=""; // sensor owner.
int register_indicator = 0;
float latitude, longitude, altitude; // to hold longitude and latitue


//chaning Web variables
int dn=0; // data_no
int d=1; // the identity of sensor, so this is first sensor, we should find a way to send it, its unqiue for all sensor.
int c=0; //co data
//int o3=0; // ozone data
//int n=0; // nitogen data
float p2 = 0;// particulate < 2.5 matter data
float p1 = 0;// particulate > 2.5 matter data
//int s =0; // so2 data
//float lo= 0; //longituted data
//float l = 0;// latitude data.
//float h= 0;// height data
//float t= 0; // sensor time UTC.
int b = 0;// battery status
float tm = 0;// temperature data
float hu= 0; //hummidity data
float di= 900; //data interval

//PM data and variables
#define DUST_SENSOR_DIGITAL_PIN_PM10  8
#define DUST_SENSOR_DIGITAL_PIN_PM25  9
unsigned long SLEEP_TIME = 30*1000; // Sleep time between reads (in milliseconds)
int val = 0;           // variable to store the value coming from the sensor
float valDUSTPM25 =0.0;
float lastDUSTPM25 =0.0;
float valDUSTPM10 =0.0;
float lastDUSTPM10 =0.0;
unsigned long duration;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
long concentrationPM25 = 0;
long concentrationPM10 = 0;
float concentration = 0;
float concentrationPM25_ugm3;
float concentrationPM10_ugm3;
int temp=273.5; //external temperature, if you can replace this with a DHT11 or better 
//float ppmvPM25;
//float ppmvPM10;

//CO data and variables
#define COPIN  A0
//int co_value=0;

//GSM SHIELD parameter
uint8_t gprsresult =0; // hold result whether gprs is sucessful or not
uint8_t webresult =7; //hold result whether web action is sucessful or not
int webstatus=0; //hold the status messge return from the web
uint8_t result=0; //holding result from calling gprs/gps function.

void setup(void) {
 //set PM pin mode.
  dht.begin(); //start DHT11 sensor
  pinMode(DUST_SENSOR_DIGITAL_PIN_PM10,INPUT);
  pinMode(DUST_SENSOR_DIGITAL_PIN_PM25,INPUT);
  restart_shield(); //start up shield.

}
 
void loop(void) {
      //register sensor before pouring data, when register, register_indicator=1
      if (! register_indicator) //if you have not registered the register.
      {
        
            
      }
       // help to hold real-time average values of pollutant data captured
        real_time_average_pm10=1;
        real_time_average_pm25=1;
        real_time_average_co=1;
        real_time_average_tem=1;
        real_time_average_hum=1;
    
    // Capture data every 1 mins and send average of captured pollutant data every 15 mins, then capture over a 24 hr period i.e 96 times
      for (int i =1; i<=96; i++){ 

        next_state = 1;
        int captured_number = 1; //hold the number of times in the 15 minustes period when data is captured.
        
        //some inout should go in here
        dn = i; //data no, let us identify which data is coming in.
        
        start_cap_time_ms =millis(); //this is the time that keep watch at 15minutes interval for sending data.
        fona.enableGPS(true); //Turn on gps, so that you can get a fix after 2minutes before requesting lonti and latituge.
        do { //Between here, capture as much value as you can and find the average.
                //start state at 1. // start reading temp and hummidity data.
        switch (next_state){
        
                //capture temperature data and humidity data
          case 1: next_state = get_temp_hum_data (next_state, captured_number); // Check for echicle entrace
          break;
          case 2: next_state = get_PM25_data (next_state, captured_number); // Check for echicle entrace
          break;
          case 3: next_state = get_PM10_data (next_state, captured_number); // Check for echicle entrace
          break;
          case 4: next_state = get_co_data (next_state, captured_number); // Check for echicle entrace
          break;
          case 41:  // case 41 is the reporting case where captured number is being updated. accurately. // only if all data has been gotten.
          if (next_state == 41){
             captured_number++; // increase the number of times pollutant data has been captured. // the last state should return case 41, which is get_PM10_data
          }
          next_state=1; // start capturing again by reverting to state 1
          break;
         
          default: //captured_number--; //next_state = clean_all_data (next_state, captured_number); // Check for echicle entrace // reduce captured number when last as being cpatured because it will be increased
          Serial.println("entered default state");
          next_state=1;
          break;
        
        }//end of swicth
         //calculate realtime average.
        // Serial.print("captured number is: ");  // we already displayed it.
         Serial.println(captured_number);
         end_cap_time_ms = millis();
        } while((end_cap_time_ms - start_cap_time_ms)< 120000); //appx 600000 Have we done 10 minutes of data capture? if yes, try transmiting data and hold on.
//        Serial.println(end_cap_time_ms - start_cap_time_ms);
        
        do{ //Between here, prepare the URL and assign all values and try to send data to the server until you are sucessful. if not save, we can save 30.
               /*  Serial.println("report of captures");
                 Serial.print("average pm25 is: "); 
                 Serial.println(real_time_average_pm25);
                 Serial.print("average pm10 is: "); 
                 Serial.println(real_time_average_pm10);
                 Serial.print("average temp is: "); 
                 Serial.println(real_time_average_tem);
                 Serial.print("average humidity is: "); 
                 Serial.println(real_time_average_hum);
                 Serial.print("average co is: "); 
                 Serial.println(real_time_average_co);
                 Serial.print("no of data captured is "); 
                 Serial.println(captured_number);*/
        //get GPS, latitutude, longitude, height, 
        static char outstr[10]; //hold the str that allow us to render longitude data properly.
        if (!get_long_lat_alt ()){ // if the gps fails // get the last correct gps from eeprom.
            Serial.println("gpsf");
             }
        /*String u ="airquality.codefornigeria.org/input_sensor_data/?d=1&c=1&o3=41&n=1"; // the way URL should be added.
          u += "&p2=";
          u += random(1, 10);
          u += "&p1=8&s=1&lo=3&l=7&h=5&t=2&b=1&tm=3&hu=4&dn=1&di=900"; */
        //String u ="airquality.codefornigeria.org/input_sensor_data/?d=1&c=1&o3=4&n=1&p2=3&p1=8&s=1&lo=3&l=7&h=5&t=1&b=1&tm=3&hu=4&dn=96&di=900"; // this what the string should be.
      /*  String u ="airquality.codefornigeria.org/input_sensor_data/?d=1&c=1&o3=41&n=1";
        u += "&p2=";
        u += random(1, 10);
        u += "&p1=8&s=1&lo=3&l=7&h=5&t=2&b=1&tm=3&hu=4&dn=1&di=900";*/
          //compose string.
        /*
        String u= "airquality.codefornigeria.org/input_sensor_data/?d=1"; // this is the url for the first sensor and the d=1
         //String u= "cfaairquality.pythonanywhere.com/input_sensor_data/?d=1"; // this is the url for the first sensor and the d=1
         u +="&c=";
         u +=map (map(real_time_average_co, 0.1, 1023.1, 10.1, 500.1), 10.0,500.0, 0.0,50.4); 
         u +="&o3=";
         u +=0;
         u +="&n=";
         u +=0;
         u +="&p2=";
         u +=real_time_average_pm25;
         u +="&p1=";
         u +=real_time_average_pm10;
         u +="&s=";
         u +=0;
         u +="&lo=";
         u +=dtostrf(longitude, 7,5, outstr );
         u +="&l=";
         u +=dtostrf(latitude, 7,5, outstr );
         u +="&h=";
         u +=dtostrf(altitude, 7,5, outstr );
         u +="&t=";
         u +=0;
         u +="&b=";
         u +=1; //read analog of powerbank
         u +="&tm=";
         u +=real_time_average_tem;
         u +="&hu=";
         u +=real_time_average_hum;
         u +="&dn=";
         u +=dn;
         u +="&di=";
         u +=900; // our data interval is 900 secs. 
        */
        //Sample test to see if data will go through.
        String url = "api.airquality.codeforafrica.org/v1/push-sensor-data/?NODE=CFA001&PIN=2";
        String data = "{\”sampling_rate\”:2,\”timestamp\”:\”2017-6-6 2:7:1\”,\”sensordatavalues\”:[{\”value\”:\”9\”,\”value_type\”:\”P1\”},{\”value\”:\”1\”,\”value_type\”:\”P2\”},{\”value\”:\”4\”,\”value_type\”:\”samples\”},{\”value\”:\”1\”,\”value_type\”:\”min_micro\”},{\”value\”:\”2\”,\”value_type\”:\”max_micro\”}]}";
        /**********************BETWEEN HERE SEND DATA TO THE SERVER/DATABASE***********************/
            //  Serial.println(u);
              unsigned long time_started = millis();
              do{
                 //first turn on GPRS
                gprsresult= send_GPS_RS_command_GgOo ('G');
                if (gprsresult==11){
                   Serial.println(F("GRsuc"));
                }
                else{
                  Serial.println(F("GRfai")); 
                }
              webstatus=5; // let us know the web status
              webresult = send_data_to_web ('w', url, data);
              if (webresult==1){
                 Serial.println("Web suc");
              //tunr off
                restart_shield();
                break; // if sucessful what are you waiting for, get outta of here
              }
              else{
                Serial.println("wb nt suc");
                Serial.print("stus ");
                Serial.print(webstatus);
                restart_shield();
              }
              //tunr off gprs
              gprsresult= send_GPS_RS_command_GgOo ('g'); 
              if (gprsresult==21){
                 Serial.println(F("GR suc"));
              }
              else{
                Serial.println(F("GR fai"));
              }
              delay(5000);
              } while ((millis() - time_started)< 240000); // try to send the data between a period of 3.5 minutes, if not sucessful, then save data to be sent later.
                
        /*******************************************************************************************/
        real_time_average_pm10=1;
        real_time_average_pm25=1;
        real_time_average_co=1;
        real_time_average_tem=1;
        real_time_average_hum=1;                                           
        
        // in between a period of 4 minutes, the GPRS sheild tries to send the data, if after unscessful attempt in 4 mins, its over, data is saved.
        
        }while(0);
        
        Serial.print("n   ");
        Serial.print(dn);
        Serial.print(" pdat");
     }
}
  
/*void restart_gprs_device ()
{
 while (!Serial);
 Serial.begin(115200);
 //Serial.println(F("FONA Starting"));
// Serial.println(F("Initializing....(May take 3 seconds)"));
 
   fonaSerial->begin(4800);
   if (! fona.begin(*fonaSerial)) {
     Serial.println(F("Couldn't find FONA"));
     while (1);
   }
   delay(4000);
   Serial.println(F("FONA ok"));
      
 }*/

uint8_t get_temp_hum_data( uint8_t next_state, uint8_t captured_num){
// READ DATA
delay(2000);// let dht11 stabilise
for (int i=1; i<6; i++) {
tm = dht.readHumidity();
hu = dht.readTemperature();
//int chk = DHT.read11(DHT11_PIN);
if (!(isnan(hu) || isnan(tm))) // humidity does not read well, but temperature reads, well, if everything is ok then read tem and hum
{
Serial.println("gvl"); 
//Serial.println(DHT.humidity); 
//Serial.println(DHT.temperature); 
//Serial.println(" "); 
//delay (2000); // No need for this delay when things are fine.
//*/

//get realtime average data for both temp and humidity and save as hu and dm.
  real_time_average_tem = ((real_time_average_tem * (captured_num -1))+ tm)/captured_num;
  real_time_average_hum = ((real_time_average_hum * (captured_num -1))+ hu)/captured_num;
//....................

  next_state = 2;
  return next_state;
 break; // break the looop. you got what you want!
}

else{ //store the value, yet read again.
//hu = DHT.humidity; // store humididty ready for web
//tm = DHT.temperature; //store temperature ready for web
Serial.println("Nv"); 
Serial.println(i); 
Serial.println("h: ");
//*/
delay (2000); // wait a while. and read again
//  real_time_average_tem = ((real_time_average_tem * (captured_num -1))+ DHT.temperature)/captured_num;
//  real_time_average_hum = ((real_time_average_hum * (captured_num -1))+ DHT.humidity)/captured_num;
  
if (i==5){
  next_state = 2;
  return next_state;}
}
  }

//Serial.println("i am out"); //done
}

//PM 2.5 data
uint8_t get_PM25_data ( uint8_t next_state, uint8_t captured_num) {
//get PM 2.5 density of particles over 2.5 μm.
  concentrationPM25=(long)getPM(DUST_SENSOR_DIGITAL_PIN_PM25);
  Serial.print("P2");
  Serial.print(concentrationPM25);
  Serial.println(" p/cf");
  concentrationPM25_ugm3 = conversion25(concentrationPM25);
  Serial.print("P2: ");
  Serial.print(concentrationPM25_ugm3);
  Serial.println(" ug/m3");
  Serial.print("\n");   
  //ppmv=mg/m3 * (0.08205*Tmp)/Molecular_mass
  //0.08205   = Universal gas constant in atm·m3/(kmol·K)
  //ppmvPM25=((concentrationPM25_ugm3) * ((0.08205*temp)/28.97));
 
  if (concentrationPM25_ugm3>0) {
       p2 = concentrationPM25_ugm3; //assging to p2 webholder for PM10
        lastDUSTPM25= p2 ;
  }
  else{
    p2 = float (lastDUSTPM25);
  }
  //get realtime average data for both PM2.5 pollutant.
  real_time_average_pm25 = ((real_time_average_pm25 * (captured_num -1))+ p2)/captured_num;
//....................
  next_state = 3;
  return next_state;

}

//PM 10 data
uint8_t get_PM10_data ( uint8_t next_state, uint8_t captured_num){
    //get PM 1.0 - density of particles over 1 μm.
  concentrationPM10=getPM(DUST_SENSOR_DIGITAL_PIN_PM10);
  Serial.print("P1: ");
  Serial.print(concentrationPM10);
  Serial.println(" p/f");
  concentrationPM10_ugm3 = conversion10(concentrationPM10);
  Serial.print("P1 ");
  Serial.print(concentrationPM10_ugm3);
  Serial.println(" ug/m3");
  Serial.print("\n");
  //ppmv=mg/m3 * (0.08205*Tmp)/Molecular_mass
  //0.08205   = Universal gas constant in atm·m3/(kmol·K)
  //ppmvPM10=((concentrationPM10_ugm3) * ((0.08205*temp)/28.97));
  
  if (concentrationPM10_ugm3>0) {
       p1 = concentrationPM10_ugm3; //assging to p2 webholder for PM10
        lastDUSTPM10= p1 ;
  }
  else{
    p1 = float (lastDUSTPM10);
  }
  //get realtime average data for both PM2.5 pollutant.
  real_time_average_pm10 = ((real_time_average_pm10 * (captured_num -1))+ p1)/captured_num;
//....................
  next_state = 4;
  return next_state;

}
uint8_t get_co_data ( uint8_t next_state, uint8_t captured_num){
c=analogRead(COPIN);//Read Gas value from analog 0
real_time_average_co = ((real_time_average_co * (captured_num -1))+ c)/captured_num;
next_state = 41;
return next_state;
}



// function used in calculating PM2.5 and PM10 data.
float conversion25(long concentrationPM25) {
  double pi = 3.14159;
  double density = 1.65 * pow (10, 12);
  double r25 = 0.44 * pow (10, -6);
  double vol25 = (4/3) * pi * pow (r25, 3);
  double mass25 = density * vol25;
  double K = 3531.5;
  return (concentrationPM25) * K * mass25;
}

float conversion10(long concentrationPM10) {
  double pi = 3.14159;
  double density = 1.65 * pow (10, 12);
  double r10 = 0.44 * pow (10, -6);
  double vol10 = (4/3) * pi * pow (r10, 3);
  double mass10 = density * vol10;
  double K = 3531.5;
  return (concentrationPM10) * K * mass10;
}


long getPM(int DUST_SENSOR_DIGITAL_PIN) {

  starttime = millis();

  while (1) {
  
    duration = pulseIn(DUST_SENSOR_DIGITAL_PIN, LOW);
    lowpulseoccupancy += duration;
    endtime = millis();
    
    if ((endtime-starttime) > sampletime_ms)
    {
    ratio = (lowpulseoccupancy-endtime+starttime)/(sampletime_ms*10.0);  // Integer percentage 0=>100
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    Serial.print("lpo:");
    Serial.print(lowpulseoccupancy);
    Serial.print("\n");
    Serial.print("rat:");
    Serial.print(ratio);
    Serial.print("\n");
    //Serial.print("PPDNS42:");
    //Serial.println(concentration);
    //Serial.print("\n");
    
    lowpulseoccupancy = 0;
    return(concentration);    
    }
  }  
}

//flush serial
void flushSerial() {
  while (Serial.available())
    Serial.read();
}
//function to restart GSM SHIELD
void restart_shield (){
  //Serial.println("ty.....");
  while (! Serial);

  Serial.begin(115200); //used basicllay to interact with fona from serial so you can take off.
  Serial.println(F("initilzng"));
//  Serial.println(F("In)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("FNF")); //08122259554
    while(1);
  }
  Serial.println(F("FONA OK"));
  // Try to enable GPRS

Serial.println("ety..");
delay(10000);

}

//fundtion to turn on gprs onand off.
int send_GPS_RS_command_GgOo (char command) {
  delay(5000);
  Serial.print(F("FONA> "));
    flushSerial();
    if (fona.available()) {
      Serial.write(fona.read());
    }
  
  switch (command){
    case 'G': {
        // turn GPRS on
        if (!fona.enableGPRS(true)){
          Serial.println(F("Fton"));
          result=1;
        }
        else{
          result=11;
        }
        break;
      }
      case 'g': {
        // turn GPRS on
        if (!fona.enableGPRS(false)){
          Serial.println(F("Ftoof"));
           result=2;
        }
        else{
          result=21;
        }
        break;
      }
          
   
    flushSerial();
  while (fona.available()) {
    Serial.write(fona.read());
  }
    delay(5000);
    return result;
    
  }
  
}

// function to send data to web.
int send_data_to_web (char command, String url, String data) {
     delay(5000);
    Serial.print(F("FONA> "));
    flushSerial();
    if (fona.available()) {
      Serial.write(fona.read());
    }
    int failed =1;
    switch (command){
    
    case 'W': {
        // Post data to website
        uint16_t statuscode;
        int16_t length;
        char url[100];
        char data[500];

        flushSerial();
        Serial.println(F("p"));
    //    Serial.println(F("U"));
    //    Serial.print(F("http://")); readline(url, 99);
        Serial.println(url);
        Serial.println(F("u"));
        readline(data, 499);
        Serial.println(data);

        Serial.println(F("**"));
        if (!fona.HTTP_POST_start(url, F("application/json"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
        failed = 0;
          break;
        }
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            Serial.write(c);
#endif

            length--;
            if (! length) break;
          }
        }
        Serial.println(F("\n****"));
        fona.HTTP_POST_end();
        break;
      }

    }
  flushSerial();
  while (fona.available()) {
  Serial.write(fona.read());
  }
  return failed;
 }
uint8_t get_long_lat_alt (){
/*if (fona.enableGPS(true)){ // turn on GPS manually, 2 minutes before you get long&latitude data, so you can get a 3D fix before requesting longitude &latitude
 }
delay(8000);*/
delay(5000);
 boolean gps_success = fona.getGPS(&latitude, &longitude, &altitude);
 if (!gps_success) {
   return 0;
   }
if (!fona.enableGPS(false)){
}
delay(5000);
  return 1;
}
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available()) {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}

