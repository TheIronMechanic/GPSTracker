#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "Adafruit_FONA.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "esp32/ulp.h"
#include "ulp_main.h"

#define FONA_RST 4
#define FONA_KEY 25
#define WakeUp 34
#define Movement 33
#define Ri 35
#define Pstat 32

char deviceName;
char replybuffer[255];
char cardinalDirection;
char phoneNumber[13] = "+41700000000";
char smsMessage[141];
char LATGPS [10];
char LAT1GPS [10];
char LONGGPS [10];
char LATGSM [10];
char LAT1GSM [10];
char LONGGSM [10];


bool debuggingMode = true;
bool ownerDetected = false;
bool motionDetected = false;
bool fix3D = false;
bool ok2Dfix = true;

float latitude;
float longitude;
float lastLatitude;
float lastLongitude;
float speed_kph;
float heading;
float altitude;
float latitudeGSM;
float longitudeGSM;
float distanceX;
float distanceY;
float distanceTotal;
float maxDistance = 0.001; //110m


int scanTime = 10; //In seconds

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");
static void init_run_ulp(uint32_t usec);

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());

    }
};

HardwareSerial *fonaSerial = new HardwareSerial(2);
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

gpio_num_t ulp_27 = GPIO_NUM_27;
gpio_num_t ulp_13 = GPIO_NUM_13;

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;

void setup() {
  pinMode(WakeUp, INPUT);
  pinMode(Movement, INPUT);
  pinMode(Ri, INPUT);
  pinMode(Pstat, INPUT);
  pinMode(FONA_KEY, OUTPUT);

  switchFona(true);
  Serial.begin(115200);
  fonaSerial->begin(115200);
  if (! fona.begin(*fonaSerial)) {
  Serial.println(F("Couldn't find FONA"));
  while (1);
  }
  
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause != ESP_SLEEP_WAKEUP_ULP) {
  printf("Not ULP wakeup\n");

  //delay(500);
  //fona.unlockSIM(PIN);
    
  delay(1000);
  
        uint8_t n = fona.getNetworkStatus();

        if (debuggingMode == true){
        Serial.print(F("Network status "));
        Serial.print(n);
        Serial.print(F(": "));
        if (n == 0) Serial.println(F("Not registered"));
        if (n == 1) Serial.println(F("Registered (home)"));
        if (n == 2) Serial.println(F("Not registered (searching)"));
        if (n == 3) Serial.println(F("Denied"));
        if (n == 4) Serial.println(F("Unknown"));
        if (n == 5) Serial.println(F("Registered roaming"));
        }

  send_at("at+cfgri=1"); //set RI pin to pulse when an sms is received
  

  getGPS();
  
  init_ulp_program();
   } else {

   if(digitalRead(Ri) == HIGH){
      
    lastLatitude = latitude;
    lastLongitude = longitude;
   
    getGPS();
    gsmLoc();

    dtostrf(latitude, 10, 7, LAT1GPS);
    dtostrf(longitude, 10, 7, LONGGPS);
    dtostrf(latitudeGSM, 10, 7, LAT1GSM);
    dtostrf(longitudeGSM, 10, 7, LONGGSM);

            for(int i = 0; i < 9; i++) {
          LATGPS[i] = LAT1GPS[i];
        }
        LATGPS[9] = '\0';

        for(int i = 0; i < 9; i++) {
          LATGSM[i] = LAT1GSM[i];
        }
        LATGSM[9] = '\0';

        sprintf(smsMessage, "Position requested! Current GPS Location: https://www.google.com/maps?q=%s,%s Current GSM location: %s,%s", LATGPS, LONGGPS, LATGSM, LONGGSM);

    if(debuggingMode == true){
      Serial.println("SMS Detected");
      } else {
        fona.sendSMS(phoneNumber, smsMessage);
        }
        
   } else {
    
   if (ulp_ADC_reading > 2480 )
    {
      motionDetected = true;
    }
    else
    {
      motionDetected = false;
    }

    if (motionDetected == true)
    {
      BLEDevice::init("");
      BLEScan* pBLEScan = BLEDevice::getScan(); //create new scan
      pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
      pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
      BLEScanResults foundDevices = pBLEScan->start(scanTime);
      for(int idx(0); idx < foundDevices.getCount(); ++idx)
        {
        std::string deviceName = foundDevices.getDevice(idx).getName();
        if(deviceName == "nRF5x") {
          ownerDetected = true;
          break;
          }else{
          ownerDetected = false;
          }
        }
      if (debuggingMode == true){
      Serial.print("Owner detected: ");
      Serial.print(ownerDetected);
      }
      
      delay(20000);
      
      if (ownerDetected = true)
      {
      if (debuggingMode == true){
      Serial.println("Owner present");
      }
      }
      else
      {
        lastLatitude = latitude;
        lastLongitude = longitude;
        
        getGPS();
        gsmLoc();

        distanceX = (lastLatitude - latitude);
        distanceY = (lastLongitude - longitude);
        distanceTotal = (sqrt(sq(distanceX) + sq(distanceY)));

        dtostrf(latitude, 10, 7, LAT1GPS);
        dtostrf(longitude, 10, 7, LONGGPS);
        dtostrf(latitudeGSM, 10, 7, LAT1GSM);
        dtostrf(longitudeGSM, 10, 7, LONGGSM);
        

        for(int i = 0; i < 9; i++) {
          LATGPS[i] = LAT1GPS[i];
        }
        LATGPS[9] = '\0';

        for(int i = 0; i < 9; i++) {
          LATGSM[i] = LAT1GSM[i];
        }
        LATGSM[9] = '\0';

        sprintf(smsMessage, "Motion Detected! Current GPS Location: https://www.google.com/maps?q=%s,%s Current GSM location: %s,%s", LATGPS, LONGGPS, LATGSM, LONGGSM);
        

        if (distanceTotal >= maxDistance){
          if (debuggingMode == true){
            Serial.println("SMS would be sent to: ");
            Serial.print(phoneNumber);
            Serial.println("Motion Detected!");
            Serial.println("current GPS location: ");
            Serial.print(latitude);
            Serial.print(" , ");
            Serial.print(longitude);
            Serial.println("localisation using GSM signal: ");
            Serial.print(latitudeGSM);
            Serial.print(" , ");
            Serial.print(longitudeGSM);
            Serial.println(smsMessage);
          } else {

            fona.sendSMS(phoneNumber, smsMessage);
          
          }
        }
      }
      }
    }
    }
   
   fona.enableGPS(false);
   fona.enableGPRS(false);
   switchFona(false);
   if (debuggingMode == true){
   printf("Entering deep sleep\n\n");
   }
  delay(100);
  start_ulp_program();
  ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
  esp_deep_sleep_start();


}

void loop() {
  // put your main code here, to run repeatedly:
}

static void init_ulp_program()
{
  esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,(ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);


  /* Configure ADC channel */
  /* Note: when changing channel here, also change 'adc_channel' constant
     in adc.S */
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_ulp_enable();
  ulp_low_threshold = 1 * (4095 / 3.3);  //1 volt
  ulp_high_threshold = 2 * (4095 / 3.3);   // 2 volts

  /* Set ULP wake up period to 100ms */
  ulp_set_wakeup_period(0, 100 * 1000);

  /* Disable pullup on GPIO15, in case it is connected to ground to suppress
     boot messages.
  */
  //  rtc_gpio_pullup_dis(GPIO_NUM_15);
  //  rtc_gpio_hold_en(GPIO_NUM_15);
} 

static void start_ulp_program()
{
  /* Start the program */
  esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);
}

void send_at(const char *at){
  if (debuggingMode == true){
  Serial.print("sending ");
  Serial.println(at);
  }
  for(char *p = (char *)at;*p;p++)Serial1.write(*p);
  fona.write(13);
  fona.write(10);
}

void switchFona(bool onoff){

  if(onoff == true && digitalRead(Pstat) == LOW){
  
  digitalWrite(FONA_KEY, LOW);
  delay(2000);
  digitalWrite(FONA_KEY, HIGH);
  delay(3000);
  if (! fona.begin(*fonaSerial)) {
  if (debuggingMode == true){
  Serial.println(F("Couldn't find FONA"));
  }
  }
  } else if(onoff == false && digitalRead(Pstat) == HIGH){
    digitalWrite(FONA_KEY, LOW);
  delay(2000);
  digitalWrite(FONA_KEY, HIGH);
  delay(3000);
  if (! fona.begin(*fonaSerial)) {
  if (debuggingMode == true){
  Serial.println(F("Couldn't find FONA"));
  }
  }   
  }
}

void getGPS(){
  fona.enableGPS(true);
  delay(5000);
  int8_t stat;
  // check GPS fix
  
 while(1){
 stat = fona.GPSstatus();
  if (stat < 0) Serial.println("Failed to query");
  if (stat == 0) Serial.println("GPS off");
  if (stat == 1) Serial.println("No fix");
  if (stat == 2) 
  {
  Serial.println("2D fix");
    if (ok2Dfix == true)
    {
    fona.getGPS(&latitude, &longitude, &speed_kph, &heading);
    altitude = 0;
    break;
    }
  }
  if (stat == 3) 
  {
    Serial.println("3D fix");
    fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
    fix3D = true;
    break;
  }
  else
  {
    fix3D = false;
  }
 delay(1000); 
 }
}

void gsmLoc() {
    fona.enableGPRS(true);
    delay(2000);
    fona.getGSMLoc(&latitudeGSM, &longitudeGSM);
    
    
}


