//Fona setup
#include "Adafruit_FONA.h"
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
// this is a large buffer for replies
//char replybuffer[255];
// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
// Hardware serial is also possible!
// HardwareSerial *fonaSerial = &Serial1;
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
int value = 0;

String g = "codeforafrica";
uint8_t webresult =7; //hold result whether web action is sucessful or not
int webstatus=0; //hold the status messge return from the web
uint8_t result=0; //holding result from calling gprs/gps function.
uint8_t gprsresult =0; // hold result whether gprs is sucessful or not

// P1 for PM10 & P2 for PM25
boolean valP1 = HIGH;
boolean valP2 = HIGH;

unsigned long starttime;
unsigned long durationP1;
unsigned long durationP2;

boolean trigP1 = false;
boolean trigP2 = false;
unsigned long trigOnP1;
unsigned long trigOnP2;

unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancyP1 = 0;
unsigned long lowpulseoccupancyP2 = 0;
unsigned long d1 = 0;  //web holder lowpulseoccupancyP1
unsigned long d2 = 0; //web holder lowpulseoccupancyP2

float ratio1 = 0;
float ratio2 = 0;
float r1 = 0; //web holder ratio1
float r2 = 0; //web holder ratio2
float hu = 0; //web holder humidty
float tm = 0; //web holder temperature
float concentration1 = 0;
float concentration2 = 0;
float pm10 = 0; //web holder PM10 ug/m3
float pm25 = 0; //web holder PM2.5 ug/m3

int count = 0; // No of Data Sent.

void setup() {
 // Serial.begin(9600); //Output to Serial at 9600 baud
  dht.begin(); //start DHT11 sensor
  restart_shield(); //start up shield.
  pinMode(8,INPUT); // Listen at the designated PIN
  pinMode(9,INPUT); //Listen at the designated PIN
  starttime = millis(); // store the start time
//  dht.begin(); // Start DHT
 delay(10); // Allow shield to chill.

}

void loop() {
  //read pins connected to ppd42ns
  valP1 = digitalRead(8); // PM10 PIN
  valP2 = digitalRead(9); // PM25 PIN

  if(valP1 == LOW && trigP1 == false){
    trigP1 = true;
    trigOnP1 = micros();
  }
  if (valP1 == HIGH && trigP1 == true){
    durationP1 = micros() - trigOnP1;
    lowpulseoccupancyP1 = lowpulseoccupancyP1 + durationP1;
    trigP1 = false;
  }
  if(valP2 == LOW && trigP2 == false){
    trigP2 = true;
    trigOnP2 = micros();
  }
  if (valP2 == HIGH && trigP2 == true){
    durationP2 = micros() - trigOnP2;
    lowpulseoccupancyP2 = lowpulseoccupancyP2 + durationP2;
    trigP2 = false;
  }
  // Checking if it is time to sample
  if ((millis()-starttime) > sampletime_ms)
  {
    ratio1 = lowpulseoccupancyP1/(sampletime_ms*10.0);                 // int percentage 0 to 100
    concentration1 = 1.1*pow(ratio1,3)-3.8*pow(ratio1,2)+520*ratio1+0.62; // spec sheet curve
    pm10= concentration1 * 0.428994220366712;
    
    ++count;
 /*   Serial.println("       ");
    Serial.print("***** Dataset No: ");
    Serial.print(count);
    Serial.println(" *****");

    Serial.print("Low pulse occupancy PM10     = ");
    Serial.println(lowpulseoccupancyP1);
    Serial.print("Ratio of PM10                = ");
    Serial.print(ratio1);
    Serial.println(" ");
    Serial.print("Concentration of PM10        = ");
    Serial.println(concentration1);
    Serial.print("PM10 ug/m3                   = ");
    Serial.println(concentration1 * 0.428994220366712); //concentrationPM10 * 0.42899422036671203 
  */
    ratio2 = lowpulseoccupancyP2/(sampletime_ms*10.0);
    concentration2 = 1.1*pow(ratio2,3)-3.8*pow(ratio2,2)+520*ratio2+0.62;
    pm25= concentration2 * 0.0020791672546494077;

   /* Serial.print("Low pulse occupancy PM25     = ");
    Serial.println(lowpulseoccupancyP2);
    Serial.print("Ratio of PM25                = ");
    Serial.print(ratio2);
    Serial.println(" ");
    Serial.print("Concentration of PM25        = ");
    Serial.println(concentration2);
    Serial.print("PM25 ug/m3                   = ");
    Serial.println(concentration2 * 0.0020791672546494077); //; concentrationPM2.5 * 0.0020791672546494077`
  */  // Resetting for next sampling
    d1= lowpulseoccupancyP1;
    d2 = lowpulseoccupancyP2;
    r1= ratio1;
    r2= ratio2;
     
  for (int i=1; i<6; i++) {
  hu = dht.readHumidity(); // take humidity data
  tm = dht.readTemperature(); // take temperature data
  //int chk = DHT.read11(DHT11_PIN);
  if (!(isnan(hu) || isnan(tm))) // humidity does not read well, but temperature reads, well, if everything is ok then read tem and hum
  {
  //do nothing if everything is well
  }
  else{  //try take readings again.
  delay (2000);
  hu = dht.readHumidity();
  tm = dht.readTemperature();
    }
  }
  static char outstr[10]; // used to hold string to format output data for web.
 
    //send data to the web.
        int q= 1; // we might send to two different domain. helps us to do this.
        do // allow us to use the single command to update both codeforafrica and codefornigeria
        {
         // String u="http://dweet.io/dweet/for/cfaairquality?co=99.45&o3=99.6&no=99.6&p2=99.6&p1=99.6&so=99";
         String u="ng.api.airquality."+ g +".org/?NODE=CFA003"; // this is the url for the first sensor and the d=1
         //String u= "cfaairquality.pythonanywhere.com/input_sensor_data/?d=1"; // this is the url for the first sensor and the d=1
        // u +="&d=";
        // u +=1;
         u +="&c=";
         u += 0.0;
        // u +="&o3=";
        // u +=0;
        // u +="&n=";
        // u +=0;
         u +="&p2=";
         u +=pm25;
         u +="&p1=";
         u +=pm10;
         //u +="&s=";
         //u +=0;
         u +="&lo=";
         u +=0.0;//dtostrf(longitude, 7,5, outstr );
         u +="&l=";
         u +=0.0;//(latitude, 7,5, outstr );
         u +="&h=";
         u +=0.0;//(altitude, 7,2, outstr );
        // u +="&t=";
        // u +=0;
         //u +="&b=";
         //u +=1; //read analog of powerbank
         u +="&tm=";
         u +=tm;
         u +="&hu=";
         u +=hu;
        // u +="&dn=";
        // u +=dn;
        // u +="&di=";
         //u +=900; // our data interval is 900 secs.
         u +="&d1=";
         u +=d1;
         u +="&d2=";
         u +=d2;
         u +="&r1=";
         u +=dtostrf(r1, 5,2, outstr );
         u +="&r2=";
         u +=dtostrf(r2, 5,2, outstr );
         
        
        /**********************BETWEEN HERE SEND DATA TO THE SERVER/DATABASE***********************/
            //  Serial.println(u);
              unsigned long time_started = millis();
              do{
                 //first turn on GPRS
                gprsresult= send_GPS_RS_command_GgOo ('G');
                if (gprsresult==11){
                   ///Serial.println(F("GRsuc"));
                }
                else{
                  ///Serial.println(F("GRfai")); 
                }
              webstatus=5; // let us know the web status
              webresult = send_data_to_web ('w', u);
              if (webresult==1){
                // Serial.println("Web suc");
              //tunr off
                // we want to also send data to codefornigeria
                restart_shield();
                break; // if sucessful what are you waiting for, get outta of here
              }
              else{
               // Serial.println("wb nt suc");
                ///Serial.print("stus ");
               /// Serial.print(webstatus);
                restart_shield();     
              }
              //tunr off gprs
              gprsresult= send_GPS_RS_command_GgOo ('g'); 
              if (gprsresult==21){
                /// Serial.println(F("GR suc"));
              }
              else{
               /// Serial.println(F("GR fai"));
              }
              delay(5000);
              } while ((millis() - time_started)< 240000); // try to send the data between a period of 3.5 minutes, if not sucessful, then save data to be sent later.
             // g="codefornigeria";
              q++; //increase
        } while (q <=1); // Send data twice, first to codeforafrica, next to codefornigeria 
        
    
    lowpulseoccupancyP1 = 0;
    lowpulseoccupancyP2 = 0;
    delay(20000);
    starttime = millis(); // store the start time
    }
   
//ok outside loop

}

//flush serial
void flushSerial() {
  while (Serial.available())
    Serial.read();
}
//turn on GPRS
int send_GPS_RS_command_GgOo (char command) {
  delay(5000);
///  Serial.print(F("FONA> "));
    flushSerial();
    if (fona.available()) {
      Serial.write(fona.read());
    }
  
  switch (command){
    case 'G': {
        // turn GPRS on
        if (!fona.enableGPRS(true)){
          ///Serial.println(F("Fton"));
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
          ///Serial.println(F("Ftoof"));
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

//function to restart GSM SHIELD
void restart_shield (){
  //Serial.println("ty.....");
  while (! Serial);

  Serial.begin(115200); //used basicllay to interact with fona from serial so you can take off.
///  Serial.println(F("initilzng"));
//  Serial.println(F("In)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
  ///  Serial.println(F("FNF")); //08122259554
    while(1);
  }
 // Serial.println(F("FONA OK"));
  // Try to enable GPRS

///Serial.println("ety..");
delay(10000);

}

// function to send data to web.
int send_data_to_web (char command, String url) {
     delay(5000);
    ///Serial.print(F("FONA> "));
    flushSerial();
    if (fona.available()) {
      Serial.write(fona.read());
    }
    int failed =1;
    switch (command){
    
    case 'w': {
    uint16_t statuscode;
    int16_t length;
    int16_t length2;
    //char h [400]={' '}; //hold Httpresponse, i dont expect to have more than 500 for this application.
   // String HttpResponseData="";
    char buf[url.length()+1]; //Increase this length to accomodate for large or long dataset.
    url.toCharArray(buf, url.length()+1); // maximum length of GET request is 2000
   // Serial.print("Request: ");
   // Serial.println(buf);

    if (!fona.HTTP_GET_start(buf, &statuscode, (uint16_t *)&length)) {
     failed =0;
      //  statuscode, length =0;
     //  If it fails, Wait a certain time, reintialize device, turn off gprs if on, then try again.
    }
    length2 = length;
    while (length > 0) {
    while (fona.available()) {
    char c = fona.read();
     // Serial.write is too slow, we'll write directly to Serial register!
      #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
        loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
        UDR0 = c;
      #else
        Serial.write(c);
      #endif
//      length--;
   //   h[length2-length]=c ;// load the response data into a string where it can be processed
      length--;
      
    }
  }
  if ( failed == 0){ // if failed occured, and showed error 603, due to network, then relax for Fona GPRS to get it self together again. because when it is on, it actually does not turn off, i dont know why
 /// Serial.print ("I fail ");
  }
  // if 603 occurs, it does this fona.HTTP_GET_END command immdeiately, which makes things work wrong, there should be a delay between the HTTP get_start command and HTTP_get_end command, when things fail.
  webstatus = statuscode; // let us know the stauts code.
  delay(8000);
  fona.HTTP_GET_end();
    }

    }
  flushSerial();
  while (fona.available()) {
  Serial.write(fona.read());
  }
  return failed;
 }

