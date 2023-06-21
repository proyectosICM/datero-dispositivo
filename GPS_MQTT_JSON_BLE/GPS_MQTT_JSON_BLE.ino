#include <EEPROM.h>

#include <ArduinoJson.h>

#include <BLEDevice.h>
#include <BLEAdvertisedDevice.h>
#include <BLE2902.h>
#include <BLEServer.h>
#include <BLEUtils.h>


#include "TimeLib.h"
tmElements_t te;  //Time elements structure
time_t unixTime;  // a time stamp

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


#define EEPROM_SIZE 100  // Definición del tamaño de la EEPROM_SIZE




#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024  // Set RX buffer to 1Kb
#define TINY_GSM_USE_GPRS true
#define SerialAT Serial1

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS

// set GSM PIN, if any
#define GSM_PIN ""

#define EEPROM_SIZE 100  // Definición del tamaño de la EEPROM_SIZE



int puesto;
int time_ant;
int time_post;

int Posc_Init_Placa = 0;

String Placa = "";

char placa_ant[8];
char placa_post[8];

String String_Name_Device = "";

unsigned long Init_Time;
unsigned long Current_Time;

// Your GPRS credentials, if any
const char apn[] = "claro.pe";  //SET TO YOUR APN
const char gprsUser[] = "Claro";
const char gprsPass[] = "Claro";

// Set phone number, if you want to test SMS
// Set a recipient phone number to test sending SMS (it must be in international format including the "+" sign)
String SMS_TARGET = "+51956020063";

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <SPI.h>
#include <SD.h>



bool deviceConnected = false;
bool oldDeviceConnected = false;
bool BLE_send = false;


BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;


#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);

Ticker tick;


// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds


#define UART_BAUD 115200
#define PIN_DTR 25
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCLK 14
#define SD_CS 13
#define LED_PIN 12
int counter, lastIndex, numberOfPieces = 24;
String pieces[24], input;



bool Scan_Gps_Status = false;
bool Mqtt_connected = false;
bool Send_Cloud = false;
bool First_Scan_gps = false;

bool GPS_active = false;


bool GPRS_INIT_VAL = true;

bool Listen_cloud = false;




TaskHandle_t TaskHandle_1;
TaskHandle_t TaskHandle_2;
TaskHandle_t TaskHandle_3;


double X_Conversion, Y_Conversion;

float lat = 0;
float lon = 0;
float speed = 0;
float alt = 0;
int vsat = 0;
int usat = 0;
float accuracy = 0;
int years = 0;
int months = 0;
int days = 0;
int hours = 0;
int mins = 0;
int secs = 0;

#define max_point_device 80


// MQTT details
const char *broker = "broker.hivemq.com";

const char *topicLed = "Datero/ESP32";
const char *topicInit = "GsmClientTest/init";
const char *topicLedStatus = "Datero/BD";


typedef struct
{
  double X_m;
  double Y_m;
  int type;
} Gps_Points_t;

Gps_Points_t Gps_Points[max_point_device];


String String_Read_SD = "";



uint32_t lastReconnectAttempt = 0;

// Funcion de eventos al conectarse y desconectarse
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};


// Funcion que recibe los datos de la aplicacion
// Esta funcion recibe los datos y si este dato es RECIBIR se envian los datos de Placa, Cantidad de dispositivos.
// Si lo que recibe es una Placa lo guarda en la memoria Flash
// Si recibe una cantidad lo guarda en la memoria Flash
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    String Val_Temp = "";
    char Data_Enviar[30];



    if (rxValue.length() > 0) {
      // Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++) {
        Val_Temp += rxValue[i];
      }
      Serial.println(Val_Temp);
      Serial.println(Val_Temp.c_str());
      if (Val_Temp.indexOf("Recibir") != -1) {

        memset(Data_Enviar, 0, sizeof(Data_Enviar));
        String_Name_Device.toCharArray(Data_Enviar, sizeof(Data_Enviar));
        Serial.print(Data_Enviar);
        pTxCharacteristic->setValue((uint8_t *)Data_Enviar, String_Name_Device.length());
        pTxCharacteristic->notify();
        delay(100);
      }
      if (BLE_send) {

        if (Val_Temp.indexOf("Puesto") != -1) {
          Serial.println("COMANDO PUESTO");

          memset(Data_Enviar, 0, sizeof(Data_Enviar));
          strcat(Data_Enviar, "Pt");
          char puesto_char[4];
          sprintf(puesto_char, "%d", puesto);
          strcat(Data_Enviar, puesto_char);
          pTxCharacteristic->setValue((uint8_t *)Data_Enviar, strlen(Data_Enviar));
          pTxCharacteristic->notify();
          delay(100);
          Serial.println(puesto);
        }

        if (Val_Temp.indexOf("Tiempo_ant") != -1) {
          Serial.println("COMANDO Tiempo Ant");

          memset(Data_Enviar, 0, sizeof(Data_Enviar));
          strcat(Data_Enviar, "T_a");
          char time_ant_char[4];
          sprintf(time_ant_char, "%d", time_ant);
          strcat(Data_Enviar, time_ant_char);
          pTxCharacteristic->setValue((uint8_t *)Data_Enviar, strlen(Data_Enviar));
          pTxCharacteristic->notify();
          delay(50);
          Serial.println(time_ant);
        }

        if (Val_Temp.indexOf("Tiempo_post") != -1) {
          Serial.println("COMANDO Tiempo post");
          
          memset(Data_Enviar, 0, sizeof(Data_Enviar));
          strcat(Data_Enviar, "T_p");
          char time_post_char[4];
          sprintf(time_post_char, "%d", time_post);
          strcat(Data_Enviar, time_post_char);
          pTxCharacteristic->setValue((uint8_t *)Data_Enviar, strlen(Data_Enviar));
          pTxCharacteristic->notify();
          delay(50);
        }

        if (Val_Temp.indexOf("Plate_post") != -1) {
          Serial.println("COMANDO placa post");

          Serial.print(placa_post);
          memset(Data_Enviar, 0, sizeof(Data_Enviar));
          strcat(Data_Enviar, "P_p");
          strcat(Data_Enviar, placa_post);
          pTxCharacteristic->setValue((uint8_t *)Data_Enviar, strlen(Data_Enviar));
          pTxCharacteristic->notify();
          delay(100);
        }


        if (Val_Temp.indexOf("Plate_ant") != -1) {
          Serial.println("COMANDO placa ant");


          Serial.print(placa_ant);
          strcat(Data_Enviar, "P_a");
          strcat(Data_Enviar, placa_ant);
          pTxCharacteristic->setValue((uint8_t *)Data_Enviar, strlen(Data_Enviar));
          pTxCharacteristic->notify();
          delay(100);
        }
      }



      if (Val_Temp.indexOf("Placa") != -1) {
        Placa = "";
        for (int Posc = 0; Posc <= 5; Posc++) {
          if (Posc == 3) {
            Placa += "-";
          }
          if (Posc < 3) {
            Placa += Val_Temp[Posc + 5];

            EEPROM.put(Posc_Init_Placa + Posc * 4, (int)Val_Temp[Posc + 5]);
            EEPROM.commit();




          } else {
            Placa += Val_Temp[Posc + 1 + 5];

            EEPROM.write(Posc_Init_Placa + Posc * 4, (int)Val_Temp[Posc + 1 + 5]);
            EEPROM.commit();
          }
        }




        Serial.print("Placa cambiada a: ");
        Serial.println(Placa);
      }




      if (Val_Temp.indexOf("Reset") != -1) {

        Serial.print("Reset Activado");
        ESP.restart();
      }
    }
  }
};



void mqttCallback(char *topic, byte *payload, unsigned int length) {


  char str[length + 1];

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  int i = 0;
  for (i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    str[i] = (char)payload[i];
  }
  str[i] = 0;  // Null termination
  Serial.println();
  //practise string
  //char json[] = "{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";

  StaticJsonDocument<256> doc;
  deserializeJson(doc, payload);

  // deserializeJson(doc,str); can use string instead of payload
  puesto = doc["posicion"];
  time_post = doc["timeUp"];
  time_ant = doc["timeDown"];

  char *pos_placaUp = strstr(str, "placaUp");

  memset(placa_post, 0, sizeof(placa_post));

  strncpy(placa_post, pos_placaUp + 10, 7);
  placa_post[7] = '\0';


  char *pos_placaDown = strstr(str, "placaDown");

  memset(placa_ant, 0, sizeof(placa_ant));

  strncpy(placa_ant, pos_placaDown + 12, 7);
  placa_post[7] = '\0';


  Serial.println(puesto);
  Serial.println(time_ant);
  Serial.println(time_post);
  Serial.println(placa_post);
  Serial.println(placa_ant);


  Listen_cloud = false;
}


void setup() {

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //disable brownout detector

  // Set console baud rate
  Serial.begin(115200);
  delay(100);
  EEPROM.begin(EEPROM_SIZE);
  delay(100);


  Read_GPS_Points();  // Leemos el device name de la memoria flash



  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  Read_Name_Device();

  Serial.print("Placa: ");
  Serial.println(Placa);


  // Set LED OFF
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(300);
  digitalWrite(PWR_PIN, LOW);



  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  if (!modem.restart()) {
    SerialMon.println("Failed to restart modem, attempting to continue without restarting");
    // return;
  }


  String name = modem.getModemName();
  delay(500);
  SerialMon.println("Modem Name: " + name);

  String modemInfo = modem.getModemInfo();
  delay(500);
  SerialMon.println("Modem Info: " + modemInfo);


  modem.sendAT("+CFUN=0 ");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" +CFUN=0  false ");
  }
  delay(200);

  /*
    2 Automatic
    13 GSM only
    38 LTE only
    51 GSM and LTE only
  * * * */
  String res;
  // CHANGE NETWORK MODE, IF NEEDED
  res = modem.setNetworkMode(2);
  if (res != "1") {
    DBG("setNetworkMode  false ");
    return;
  }
  delay(200);


  modem.sendAT("+CFUN=1 ");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" +CFUN=1  false ");
  }
  delay(200);

  SerialAT.println("AT+CGDCONT?");
  delay(500);

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }


  delay(2000);
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);


  Setting_BLE_APP();


  while (!First_Scan_gps) {
    Scanning_GPS();
  }




  // Scan_Gps_Status = true;



  xTaskCreatePinnedToCore(
    Task_Scan_Gps,
    "Task_Scan_Gps",
    4096,
    NULL,
    2,
    &TaskHandle_2,
    1);


  xTaskCreatePinnedToCore(
    Task_Loop_BLE,
    "Task_Loop_BLE",
    8192,
    NULL,
    4,
    &TaskHandle_1,
    0);




  xTaskCreatePinnedToCore(
    Task_Send_Cloud,
    "Task_Send_Cloud",
    8192,
    NULL,
    1,
    &TaskHandle_3,
    1);


  Serial.println("Terminando el void SETUP");
}

void loop() {

  if (Send_Cloud) {
    Serial.println("Entrando al bucle principal verdadero");
    // Make sure we're still registered on the network
    if (!modem.isNetworkConnected()) {
      SerialMon.println("Network disconnected");
      if (!modem.waitForNetwork(180000L, true)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isNetworkConnected()) {
        SerialMon.println("Network re-connected");
      }

#if TINY_GSM_USE_GPRS
      // and make sure GPRS/EPS is still connected
      if (!modem.isGprsConnected()) {
        SerialMon.println("GPRS disconnected!");
        SerialMon.print(F("Connecting to "));
        SerialMon.print(apn);
        if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
          SerialMon.println(" fail");
          delay(10000);
          return;
        }
        if (modem.isGprsConnected()) {
          SerialMon.println("GPRS reconnected");
        }
      }
#endif
    }

    if (!mqtt.connected()) {
      SerialMon.println("=== MQTT NOT CONNECTED ===");
      // Reconnect every 10 seconds
      uint32_t t = millis();
      if (t - lastReconnectAttempt > 10000L) {
        lastReconnectAttempt = t;
        if (mqttConnect()) {
          lastReconnectAttempt = 0;
        }
      }
      Send_Cloud = true;
      Scan_Gps_Status = false;

      delay(100);
      return;
    } else {
      Serial.println("ESTOY CONECTADO AL MQTT Y QUERO ENVIAR");
      Send_Cloud = false;
      Scan_Gps_Status = true;
      Listen_cloud = true;
      BLE_send = true;
      char JsonInformation[128];
      StaticJsonDocument<128> doc;

      doc["placa"] = Placa;
      //doc["latitud"] = round(lat * 1000000) / 1000000;
      doc["latitud"] = -12.123456;
      //doc["longitud"] = round(lon * 1000000) / 1000000;
      doc["longitud"] = -76.185621;
      doc["velocidad"] = int(speed);
      doc["fecha"] = unixTime;


      size_t n = serializeJson(doc, JsonInformation);

      //    Serial.println(JsonInformation);

      mqtt.publish(topicLedStatus, JsonInformation, n);
    }

    //  mqtt.loop();
    Serial.println("Termino el loop principal");
  }
}


void Setting_GPRS(void) {
}

void Task_Loop_BLE(void *pvParameters) {

  for (;;) {  // A Task shall never return or exit.
    Advertising_Set_BLE();
    vTaskDelay(pdMS_TO_TICKS(1000L));
  }
}


void Task_Scan_Gps(void *pvParameters) {

  for (;;) {
    if (Scan_Gps_Status) {
      Scanning_GPS();
      /*
      Scan_Gps_Status = false;  //borrar esto
      Send_Cloud = true;        // borrar esto
      */
      Serial.println("Estado de Gps activado");
    }
    vTaskDelay(pdMS_TO_TICKS(1000L));
  }
}




void Task_Send_Cloud(void *pvParameters) {
  for (;;) {
    if (Listen_cloud && Send_Cloud == false) {
      BLE_send = true;
      Serial.println("Estoy en el bucle de escuchar a la nube");
      mqtt.loop();
    }
    vTaskDelay(pdMS_TO_TICKS(300L));
  }
}


boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
  // boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

  if (status == false) {
    SerialMon.println(" fail");

    return false;
  }

  SerialMon.println(" success");
  // mqtt.publish(topicInit, "GsmClientTest started");
  mqtt.subscribe(topicLed);

  Serial.println("ESTOY SUSCRITO AL TOPICO");
  return mqtt.connected();
}

void Scanning_GPS(void) {
  if (!GPS_active) {
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) {
      SerialMon.println(" SGPIO=0,4,1,1 false ");
    }

    modem.enableGPS();
    delay(15000);
  }

  for (int8_t i = 15; i; i--) {
    SerialMon.println("Requesting current GPS/GNSS/GLONASS location");
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, &years, &months, &days, &hours, &mins, &secs)) {
      te.Second = secs;
      te.Hour = hours;  //11 pm
      te.Minute = mins;
      te.Day = days;
      te.Month = months;
      te.Year = years - 1970;  //Y2K, in seconds = 946684800UL
      unixTime = makeTime(te) - 18000UL;
      setTime(unixTime);

      /*
      SerialMon.println("Year: " + String(year()) + "\tMonth: " + String(month()) + "\tDay: " + String(day()));
      SerialMon.println("Hour: " + String(hour()) + "\tMinute: " + String(minute()) + "\tSecond: " + String(second()));
      */
      First_Scan_gps = true;
      Scan_Gps_Status = false;
      Send_Cloud = true;
      GPS_active = true;


      break;
    } else {
      delay(2000);
      Scan_Gps_Status = true;
      Send_Cloud = false;
      GPS_active = false;
      SerialMon.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 5s.");
    }
  }

  if (!GPS_active) {

    SerialMon.println("Disabling GPS");
    modem.disableGPS();

    // Set SIM7000G GPIO4 LOW ,turn off GPS power
    // CMD:AT+SGPIO=0,4,1,0
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,0");
    if (modem.waitResponse(10000L) != 1) {
      SerialMon.println(" SGPIO=0,4,1,0 false ");
    }
  }
  vTaskDelay(pdMS_TO_TICKS(2000L));
}


void First_Scanning_GPS(void) {
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println(" SGPIO=0,4,1,1 false ");
  }

  modem.enableGPS();
  delay(15000);

  for (int8_t i = 15; i; i--) {
    SerialMon.println("Requesting current GPS/GNSS/GLONASS location");
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, &years, &months, &days, &hours, &mins, &secs)) {
      te.Second = secs;
      te.Hour = hours;  //11 pm
      te.Minute = mins;
      te.Day = days;
      te.Month = months;
      te.Year = years - 1970;  //Y2K, in seconds = 946684800UL
      unixTime = makeTime(te) - 18000UL;
      setTime(unixTime);

      /*
      SerialMon.println("Year: " + String(year()) + "\tMonth: " + String(month()) + "\tDay: " + String(day()));
      SerialMon.println("Hour: " + String(hour()) + "\tMinute: " + String(minute()) + "\tSecond: " + String(second()));
      */
      First_Scan_gps = true;
      Scan_Gps_Status = false;
      Send_Cloud = true;


      break;
    } else {
      delay(2000);
      Scan_Gps_Status = true;
      Send_Cloud = false;
      SerialMon.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 5s.");
    }
  }

  SerialMon.println("Disabling GPS");
  modem.disableGPS();

  // Set SIM7000G GPIO4 LOW ,turn off GPS power
  // CMD:AT+SGPIO=0,4,1,0
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println(" SGPIO=0,4,1,0 false ");
  }
  vTaskDelay(pdMS_TO_TICKS(3000L));
}

void Read_GPS_Points(void) {
}


void readFile(fs::FS &fs, const char *path) {
  SerialMon.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    SerialMon.println("Failed to open file for reading");
    return;
  }

  SerialMon.print("Read from file: ");
  while (file.available()) {
    String_Read_SD = file.read();
  }
  file.close();
}

void Advertising_Set_BLE(void) {
  if (deviceConnected) {
    // pTxCharacteristic->setValue(&txValue, 1);
    //  pTxCharacteristic->notify();
    delay(10);  // bluetooth stack will go into congestion, if too many packets are sent
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

void Setting_BLE_APP(void) {
  Serial.println("Inicio BLE APP");
  // Create the BLE Device
  BLEDevice::init(std::string(String_Name_Device.c_str()));
  Serial.println("Creando servidor APP");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  Serial.println("Creando servicio");
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  Serial.println("Adjuntando caracteristica");
  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY);
  Serial.println("Añadiento modulo");

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks());
  Serial.println("Enciende app");
  // Start the service
  pService->start();

  // Start advertising
  Serial.println("Start advertising");

  pServer->getAdvertising()->start();
  // Serial.println("Waiting a client connection to notify...");
}


void Read_Name_Device(void) {
  String_Name_Device = "Combi ";

  for (int Posc = 0; Posc <= 5; Posc++) {
    int String_num_Read = EEPROM.read(Posc_Init_Placa + Posc * 4);

    if (String_num_Read <= 122 && String_num_Read >= 48) {

      Placa += char(String_num_Read);
      String_Name_Device += char(String_num_Read);
      if (Posc == 2) {
        Placa += "-";
        String_Name_Device += "-";
      }
    } else {
      Placa = "FFF-FFF";
      String_Name_Device += Placa;
      break;
    }
  }
  Serial.println(String_Name_Device);
}


void conversionUTM(float lati, float longi) {
  /*!
   * Transformación de las coordenadas geográficas a UTM
   */
  /// Sobre la geometría del delipsoide WGS84
  double a = 6378137.0;
  double b = 6356752.3142;

  //  float e = sqrt((a*a) + (b*b))/a; ///< Excentricidad.
  double e = sqrt((a * a) - (b * b)) / b;  ///< Segunda excentricidad.
  double e2 = e * e;                       ///< al cuadrado. Usaremos esta directamente.

  double c = a * a / b;  ///< Radio Polar de Curvatura.

  /// Sobre la longitud y latitud. Conversión de grados decimales a radianes.

  /*!
   * Cálculo del signo de la longitud:
   *      - Si la longitud está referida al Oeste del meridiano de Greenwich, 
   *        entonces la longitud es negativa (-).
   *      - Si la longitud está referida al Este del meridiano de Greenwich,
   *        entonces la longitud es positiva 8+).
   */

  double latRad = lati * PI / 180.0;   ///< Latitud en Radianes.
  double lonRad = longi * PI / 180.0;  ///< Longitud en Radianes.

  /// Sobre el huso.
  float huso = (longi / 6) + 31;  ///< Nos interesa quedarnos solo con la parte entera.
  int h = int(huso);

  int landa0 = h * 6 - 183;                        ///< Cálculo del meridiano central del huso en radianes.
  double Dlanda = lonRad - (landa0 * PI / 180.0);  ///< Desplazamiento del punto a calcular con respecto al meridiano central del huso.

  /*!
   * Ecuaciones de Coticchia-Surace para el paso de Geográficas a UTM (Problema directo);
   */

  /// Cálculo de Parámetros.
  double coslatRad = cos(latRad);
  double coslatRad2 = coslatRad * coslatRad;

  double A = coslatRad * sin(Dlanda);
  double xi = 0.5 * log((1 + A) / (1 - A));
  double n = atan(tan(latRad) / cos(Dlanda)) - latRad;
  double v = (c / sqrt(1 + e2 * coslatRad2)) * 0.9996;
  double z = (e2 / 2.0) * xi * xi * coslatRad2;
  double A1 = sin(2 * latRad);
  double A2 = A1 * coslatRad2;
  double J2 = latRad + (A1 / 2.0);
  double J4 = (3.0 * J2 + A2) / 4.0;
  double J6 = (5.0 * J4 + A2 * coslatRad2) / 3.0;
  double alf = 0.75 * e2;
  double bet = (5.0 / 3.0) * alf * alf;
  double gam = (35.0 / 27.0) * alf * alf * alf;
  double Bfi = 0.9996 * c * (latRad - alf * J2 + bet * J4 - gam * J6);

  /*! 
   * Cálculo final de coordenadas UTM
   */
  SerialMon.println(" Las coordenadas GPS que se van a transformar son: ");
  SerialMon.print(" Latitud: ");
  SerialMon.println(lati, 5);
  SerialMon.print(" Longitud: ");
  SerialMon.println(longi, 5);

  SerialMon.println(" Coordenadas UTM actuales: ");
  X_Conversion = xi * v * (1 + (z / 3.0)) + 500000; /*!< 500.000 es el retranqueo que se realiza en cada huso sobre el origen de
  coordenadas en el eje X con el objeto de que no existan coordenadas negativas. */
  SerialMon.print(" X = ");
  SerialMon.print(X_Conversion, 5);
  SerialMon.println(" (m)");
  Y_Conversion = n * v * (1 + z) + Bfi; /*!< En el caso de latitudes al sur del ecuador, se sumará al valor de Y 10.000.000
  para evitar coordenadas negativas. */
  SerialMon.print(" Y = ");
  SerialMon.print(Y_Conversion, 5);
  SerialMon.println(" (m)");
}