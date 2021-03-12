#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//#include <esp_now.h>
/////////////////////////////////////////////////////////////////////////////////////////
// Webserver
#include <ESPAsyncWebServer.h>
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


/////////////////////////////////////////////////////////////////////////////////////////
// Files (used for Web and History
#include <SPIFFS.h>
#include <ArduinoJson.h>
SemaphoreHandle_t xJsonDoc = xSemaphoreCreateMutex();
DynamicJsonDocument doc(20480);
DynamicJsonDocument curData(2048);
SemaphoreHandle_t xConfigDoc = xSemaphoreCreateMutex();
DynamicJsonDocument config(2048);
bool configChange = false;

/////////////////////////////////////////////////////////////////////////////////////////
// WIFI
#include <WiFi.h>
#include <DNSServer.h>
#define WIFI_NETWORK "WIFI NETWORK NAME"
#define WIFI_PASSWORD "WIFI PASSWORD"
#define WIFI_TIMEOUT_MS 30000 // 30 second WiFi connection timeout
const byte DNS_PORT = 53;
DNSServer dnsServer;
char* defaultAP = "Smoker Controller";
char* defaultApPassword = "Smoke Meats";
bool wifiApMode = false;
bool wifiConfigChange = false;

/////////////////////////////////////////////////////////////////////////////////////////
// PID
#include <PID_v1.h>
double Setpoint = 80;
double Output = 0;
double Input;
double pidTuneP = 2;
double pidTuneI = 5;
double pidTuneD = 1;
int targetMode = AUTOMATIC;
PID myPID(&Input, &Output, &Setpoint,pidTuneP,pidTuneI,pidTuneD,P_ON_M, DIRECT);

/////////////////////////////////////////////////////////////////////////////////////////
// Fan
#define fanChannel 7 //LEDc PWM Channel
#define fanPin 33
int fanPWMFreq = 25000;
double fanIn0 = 0;
double fanIn100 = 100;
double fanOut0 = 0;
double fanOut100 = 100;
double fanOut;


/////////////////////////////////////////////////////////////////////////////////////////
// Servo
#include <Servo.h>
Servo servo;
double servoIn0 = 0;
double servoIn100 = 100;
double servoOut0 = 0;
double servoOut100 = 180;
double servoOut;

/////////////////////////////////////////////////////////////////////////////////////////
// mDNS & Hostname
#include <ESPmDNS.h>
const char * hostName = "esp32-smoker";

/////////////////////////////////////////////////////////////////////////////////////////
// NTP
#include <NTPClient.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
bool timeSet = false;

/////////////////////////////////////////////////////////////////////////////////////////
// Inputs
//// Temperature
#include <Adafruit_MAX31856.h>
SemaphoreHandle_t xFiltTempMutex = xSemaphoreCreateMutex();
const int numSensors = 4;
float filteredTemps[numSensors];
float filteredCJs[numSensors];
float filteredFaults[numSensors];
float filterTime[numSensors];
float tempDeviation[numSensors];
//// Voltage
const int inputVoltPin = 35;
const int inputVoltR1 = 910;
const int inputVoltR2 = 100;
float InputVolts;

/////////////////////////////////////////////////////////////////////////////////////////
// History
SemaphoreHandle_t xHistoryMutex = xSemaphoreCreateMutex();
int historyIndex = 0;
const int historyValues = 100;
int historyCollectInterval = 15; // seconds between history collection
unsigned long historyTime[historyValues];
unsigned long historyMillis[historyValues];
float TemperatureHistory[4][historyValues];
float outputHistory[historyValues];
float setpointHistory[historyValues];
int faultHistory[4][historyValues];


/////////////////////////////////////////////////////////////////////////////////////////
// Alarms
#include "pitches.h"
#define buzzerPin 32
#define buzzerChannel 8 //LEDc PWM Channel
float probeTarget[numSensors];
bool probeTgtAlm[numSensors];
bool probeTgtAck[numSensors];
bool probeTgtAct[numSensors];
bool pitTempHighAlm;
bool pitTempHighAck;
bool pitTempLowAlm;
bool pitTempLowAck;

bool lidOpenDetect;
bool lidOpen = false;
float lidOpnPctDeviation;
int lidOpenTimeLeft = -1;



//int probe1Index = 0;
//int probe2Index = 1;


TaskHandle_t xSampleTemperatures = NULL;
TaskHandle_t xPidControl = NULL;
TaskHandle_t xKeepWiFiAlive = NULL;
TaskHandle_t xSoundBuzzer = NULL;
TaskHandle_t xWaitForWifi = NULL;
TaskHandle_t xNtpUpdate = NULL;
TaskHandle_t xCollectHistory = NULL;
TaskHandle_t xWriteHistToFlash = NULL;
TaskHandle_t xWriteConfigToFlash = NULL;
TaskHandle_t xSendWebEvents = NULL;

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  if(!SPIFFS.begin()){
    Serial.println("[W] SPIFFS Error");
    return;
  }
  else{
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
    while(file){
        Serial.print("[FILE] ");
        Serial.println(file.name());
        file = root.openNextFile();
    }
  }

  //Config Restore
  Serial.println("[FILE] Read Config from file");
  xSemaphoreTake( xConfigDoc, portMAX_DELAY );
  File configFile = SPIFFS.open("/config.json", FILE_READ);
  deserializeJson(config, configFile);
  configFile.close();
  for(int i=0; i< numSensors; i++){
    probeTarget[i] = config["probeTarget"+String(i)];
    filterTime[i] = config["filterTime"+String(i)];
    probeTgtAlm[i] = config["probeTgtAlm"+String(i)];
  }
  Setpoint = config["setpoint"];
  Output = config["output"];
  targetMode = config["targetMode"];
  pidTuneP = config["pidTuneP"];
  pidTuneI = config["pidTuneI"];
  pidTuneD = config["pidTuneD"];
  servoIn0 = config["servoIn0"];
  servoIn100 = config["servoIn100"];
  servoOut0 = config["servoOut0"];
  servoOut100 = config["servoOut100"];
  fanIn0 = config["fanIn0"];
  fanIn100 = config["fanIn100"];
  fanOut0 = config["fanOut0"];
  fanOut100 = config["fanOut100"];
  lidOpenDetect = config["lidOpenDetect"];
  lidOpnPctDeviation = config["lidOpnPctDeviation"];
  xSemaphoreGive( xConfigDoc );
  
  xTaskCreate(
    sampleTemperatures
    ,  "Sample Temperatures"   // A name just for humans
    ,  3072  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  20  // Priority
    ,  &xSampleTemperatures 
    );

  xTaskCreate(
    pidControl
    ,  "pidControl"   // A name just for humans
    ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  18  // Priority
    ,  &xPidControl 
    );
  const int collectHistStackSize = 4096;
  xTaskCreate(
    collectHistory,
    "collectHistory",  // Task name
    collectHistStackSize,             // Stack size (bytes)
    NULL,             // Parameter
    15,                // Task priority
    &xCollectHistory             // Task handle
    ); 

  xTaskCreate(
    sendWebEvents,
    "sendWebEvents",  // Task name
    4096,             // Stack size (bytes)
    NULL,             // Parameter
    3,                // Task priority
    &xSendWebEvents             // Task handle
    ); 

  xTaskCreate(
    keepWiFiAlive,
    "keepWiFiAlive",  // Task name
    4096,             // Stack size (bytes)
    NULL,             // Parameter
    5,                // Task priority
    &xKeepWiFiAlive             // Task handle
    );

  xTaskCreate(
    waitForWifi,
    "waitForWifi",  // Task name
    4096,             // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority
    &xWaitForWifi             // Task handle
    );  

  xTaskCreate(
    writeHistToFlash,
    "writeHistToFlash",  // Task name
    4096,             // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority
    &xWriteHistToFlash             // Task handle
    );  

  xTaskCreate(
    writeConfigToFlash,
    "writeConfigToFlash",  // Task name
    4096,             // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority
    &xWriteConfigToFlash             // Task handle
    );  

  xTaskCreate(
    Stacks,
    "Stacks",  // Task name
    1024,             // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority
    NULL             // Task handle
    ); 

  xTaskCreate(
    soundBuzzer,
    "soundBuzzer",  // Task name
    2048,             // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority
    &xSoundBuzzer             // Task handle
    ); 
}

void loop()
{
  vTaskSuspend(NULL);
}

void Stacks(void *pvParameters)  // This is a task.
{
  for (;;) {
    vTaskDelay(10000 / portTICK_PERIOD_MS);
      Serial.println();
      for (int probeIndex = 0;probeIndex<numSensors;probeIndex++) {
        Serial.print("[PV] Probe Index: ");
        Serial.print(probeIndex);
        Serial.print(" Cold Junction Temp: ");
        Serial.print(filteredCJs[probeIndex]);
        Serial.print(" Thermocouple Temp: ");
        Serial.print(filteredTemps[probeIndex]);
        if (filteredFaults[probeIndex]) {
          Serial.print(" Thermocouple Fault: ");
          Serial.print(filteredFaults[probeIndex]);
        }
        Serial.println();
      }

      Serial.print("[PID] Computed Output: ");
      Serial.println(Output);
      
      Serial.print("[JSON] History Doc Memory Usage: ");
      Serial.println(doc.memoryUsage());

      Serial.print("[JSON] Current Data Memory Usage: ");
      Serial.println(curData.memoryUsage());

      Serial.print("[MEM] Memory Free: ");
      Serial.println(ESP.getFreeHeap());

      Serial.print("[FILE] Total Space: ");
      Serial.println(SPIFFS.totalBytes());
      
      Serial.print("[FILE] Total Space Used: ");
      Serial.println(SPIFFS.usedBytes());
      
      Serial.println("[STCK]Process  Priority  Stack Free");
      
      Serial.print(  "  Sample Temp   ");
      Serial.print(uxTaskPriorityGet(xSampleTemperatures));
      Serial.print("     ");
      Serial.println(uxTaskGetStackHighWaterMark( xSampleTemperatures ));
      
      Serial.print(  "  PID Control   ");
      Serial.print(uxTaskPriorityGet(xPidControl));
      Serial.print("     ");
      Serial.println(uxTaskGetStackHighWaterMark( xPidControl ));
      
      Serial.print(  "  Collect Hist  ");
      Serial.print(uxTaskPriorityGet(xCollectHistory));
      Serial.print("     ");
      Serial.println(uxTaskGetStackHighWaterMark( xCollectHistory ));

      Serial.print(  "  Wrt Hist Flash");
      Serial.print(uxTaskPriorityGet(xWriteHistToFlash));
      Serial.print("     ");
      Serial.println(uxTaskGetStackHighWaterMark( xWriteHistToFlash ));

      Serial.print(  "  Wrt Cnfg Flash");
      Serial.print(uxTaskPriorityGet(xWriteConfigToFlash));
      Serial.print("     ");
      Serial.println(uxTaskGetStackHighWaterMark( xWriteConfigToFlash ));
      
      Serial.print(  "  Web Event     ");
      Serial.print(uxTaskPriorityGet(xSendWebEvents));
      Serial.print("     ");
      Serial.println(uxTaskGetStackHighWaterMark( xSendWebEvents ));

      Serial.print(  "  Keep WiFi     ");
      Serial.print(uxTaskPriorityGet(xKeepWiFiAlive));
      Serial.print("     ");
      Serial.println(uxTaskGetStackHighWaterMark( xKeepWiFiAlive ));
      
      Serial.print(  "  NTP Update    ");
      Serial.print(uxTaskPriorityGet(xNtpUpdate));
      Serial.print("     ");
      Serial.println(uxTaskGetStackHighWaterMark( xNtpUpdate ));
      
      Serial.print(  "  Sound Buzzer  ");
      Serial.print(uxTaskPriorityGet(xSoundBuzzer));
      Serial.print("     ");
      Serial.println(uxTaskGetStackHighWaterMark( xSoundBuzzer ));
      
      Serial.print("[UPTIME] ");
      Serial.println(millis()/1000);
      
  }
}
