/*18/09/2024
  Version LTE-M
  SIM7000G carte LILYGO_SIM7000G
  https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G

  ********************************************
  Au premier lancement nouvelle carte SIM7000G
  il faut mini les 2 premiers + GPS:
  modem.setPreferredMode(1); 1 CAT-M (+CMNB)
  modem.setNetworkMode(38);  38 LTE only (+CNMP)

  modem.getNetworkCurrentMode (+CNSMOD)PhC
  modem.getNetworkSystemMode (+CNSMOD)
  modem.setNetworkSystemMode (+CNSMOD=0/1 auto report)

  si GPS utilisé sur carte LILYGO
  Alimentation de l'antenne GPS
  sendat=+CGPIO=0,48,1,1
  ********************************************

  TinyGsmClient.h
  librairie TinyGSM revue PhC

  A FAIRE
  inclure les modifications de la version 10.11.5 vers 
  version 0.12.0 revue PhC

  */

/* A finir
Alarme MQTT, GPRS,GPS
ismqttconnect?
senddata
*/

/*
Compilation ESP32 2.0.17
Arduino IDE VSCODE : 1116069 85%, 55652 16% sur PC
Arduino IDE 1.8.19 :  sur raspi
*/


#include <Arduino.h>

String ver     = "V1-00";
int Magique    = 2;

#define LILYGO_SIM7000G // not define for SIM7000G board only

#define TINY_GSM_MODEM_SIM7000
#define Exploitation           // selection des données serveur dans fichier credentials

#include <Battpct.h>
#include "defs.h"
#include <TinyGsmClient.h>         // librairie TinyGSM revue PhC 10.12.0
#include <PubSubClient.h>
#include <TimeAlarms.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <FS.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>
// #include <Update.h>               //update
// #include <CRC32.h>                //update
#include "passdata.h"
#include <ArduinoJson.h>
#include <credentials_tpcf.h>

#define Serial Serial
#define SerialAT Serial2
#define TINY_GSM_DEBUG Serial
#define TINY_GSM_USE_GPRS true
// #define TINY_GSM_USE_WIFI false
// #define GSM_PIN "1234"

String  webpage = "";
#define ServerVersion "1.0"
bool    SPIFFS_present = false;
#include "CSS.h"               // pageweb

#define MODEM_TX            27
#define MODEM_RX            26
#define MODEM_PWRKEY        4
#define MODEM_DTR           32
#define MODEM_RI            33
#define MODEM_FLIGHT        25
#define MODEM_STATUS        34
#define SD_MISO             2    // Carte SD
#define SD_MOSI             15   // Carte SD
#define SD_SCLK             14   // Carte SD
#define SD_CS               13   // Carte SD
#define LED_PIN             12
#define LED_EXTERNE_PIN     22   // Led indicateur vers l'exterieur
#define PinBattProc         35   // liaison interne carte Lolin32 adc
#define PinAlim             36   // mesure tension alimentation SOLAR IN
#define PinReset            18   // 13   // Reset Hard ESP32
#define NTPServer "pool.ntp.org"
#define uS_TO_S_FACTOR      1000000ULL  /* Conversion factor for micro seconds to seconds */
#define FORMAT_SPIFFS_IF_FAILED true // format SPIFFS si lecture impossible
#define nSample (1<<4)                   // nSample est une puissance de 2, ici 16 (4bits)
unsigned int adc_hist[5][nSample];       // tableau stockage mesure adc, 0 Batt, 1 Proc, 2 USB, 3 24V, 5 Lum
unsigned int adc_mm[5];                  // stockage pour la moyenne mobile

unsigned long debut      = 0;               // pour decompteur temps wifi
char fileconfig[12]      = "/config.txt";   // fichier en SPIFFS contenant structure config
char filecalendrier[13]  = "/filecal.csv";  // fichier en SPIFFS contenant le calendrier de circulation
char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
char filelog[9]          = "/log.txt";      // fichier en SPIFFS contenant le log
char filelumlut[13]      = "/lumlut.txt";   // fichier en SPIFFS LUT luminosité
char filePhoneBook[8]    = "/pb.txt";       // fichier contenant liste N°tel autorisé

char PB_list[10][30];                       // PB liste en ram

const String soft = "ESP32_Tracker.ino.d32";// nom du soft

bool FlagReset = false;
byte Accu = 0;                           // accumulateur vitesse(bascule TimerLent/TimerRapide)
float lat, lon, course, speed;
long   VBatterieProc    = 0;             // Tension Batterie Processeur
long   VAlim            = 0;             // Tension Alimentation
String Sbidon 		= "";                  // String texte temporaire
String Rmessage = "";                    // Receive message
String message = "";                     // Reponse message envoyé
String opmessage = "";                   // message vers sortie (Serial et SD)
String fl = "\n";                        // saut de ligne SMS
String Id ;                              //  Id du materiel sera lu dans config
String SDfilename;

bool FlagAlarmeTension     = false;
bool FlagAlarmeGprs        = false;
bool FlagAlarmeMQTT        = false;
bool FlagAlarmeGps         = false;
bool AlarmeMQTT            = false;

bool FlagLastAlarmeTension = false;
bool FlagLastAlarmeGprs    = false;
bool FlagLastAlarmeMQTT    = false;
bool FlagLastAlarmeGps     = false;
bool FlagMQTTReceive       = false;
bool FlagSDCardPresent     = false;
bool Online                = false; // true si network && Gprs && mqtt && subscribe
bool lastOnline            = false; // memorise dernier etat de Online
bool flagRcvMQTT           = false; // true si reception MQTT longueur>0, false longueur = 0
bool FlagSetTime           = false; // flag set time valide
int CoeffTension[4];                // Coeff calibration Tension
int CoeffTensionDefaut     = 7000;  // Coefficient par defaut
int BlinkerInterval        = 1000;  // ms blinker lent/rapide

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PhonebookEntry Phone;

PubSubClient mqttClient(client);

WebServer server_wifi(80);
File UploadFile;

struct  config_t           // Structure configuration sauvée en EEPROM
{
  int     magic;           // num magique
  bool    tracker;         // Tracker ON/OFF
  int     trapide;         // timer send data rapide (roulage)
  int     tlent;           // timer send data lent (arret)
  int     vtransition;     // vitesse transition arret/roulage
  int     timeoutWifi;     // tempo coupure Wifi si pas de mise a jour (s)
  char    apn[11];         // APN
  char    gprsUser[11];    // user for APN
  char    gprsPass[11];    // pass for APN
  char    mqttServer[26];  // Serveur MQTT
  char    mqttUserName[11];// MQTT User
  char    mqttPass[16];    // MQTT pass
  char    sendTopic[2][13];// output to server
  char    recvTopic[2][13];// input from server
  int     mqttPort;        // Port serveur MQTT
  int     hete;            // decalage Heure été UTC
  int     hhiver;          // decalage Heure hiver UTC
  char    Idchar[11];      // Id
  bool    sendSMS;         // Autorisation envoie SMS
  bool    logSDcard;       // log sur carte SD
  uint16_t keepAlive;      // Paramètre keep alive de Pubsubclient
} ;
config_t config;

char willTopic[7];

int N_Y, N_M, N_D, N_H, N_m, N_S; // variable Date/Time temporaire

int NbrResetModem = 0;              // Nombre de fois reset modem, remise à 0 signal vie
String dataToPublish;   // Holds your field data.

Ticker ADC;                // Lecture des Adc

Ticker Blinker;          // LED externe Clignotant

AlarmId loopPrincipale;    // boucle principale
AlarmId first;             // premier Send data
AlarmId Send;              // timer send data
// AlarmId Svie;              // Signal de Vie

//---------------------------------------------------------------------------
// newtime = "" mise à l'heure NTP
// newtime = "YY/MM/DD,hh:mm:ss" mise à l'heure reçue
bool MajHeure(String newtime = "");
//---------------------------------------------------------------------------
void Change(bool on);
//---------------------------------------------------------------------------
void setup() {
  message.reserve(300); // texte des reponses
  setCpuFrequencyMhz(80);// 30mA, a la place de 240MHz 65mA par defaut
  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print(F("Version Soft : ")), Serial.println(ver);

  Serial.print("Xtal F:"),Serial.println(getXtalFrequencyMhz());
  Serial.print("CPU  F:"),Serial.println(getCpuFrequencyMhz());
  
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) { // Format la première fois utilise SPIFFS
    Serial.println(F("SPIFFS initialisation failed..."));
    SPIFFS_present = false;
  }
  else {
    Serial.println(F("SPIFFS initialised... file access enabled..."));
    SPIFFS_present = true;
  }
  pinMode(LED_EXTERNE_PIN, OUTPUT);
  digitalWrite(LED_EXTERNE_PIN, HIGH);

  // Init SD card
  pinMode (SD_SCLK, INPUT_PULLUP); // OBLIGATOIRE
  pinMode (SD_MISO, INPUT_PULLUP);
  pinMode (SD_MOSI, INPUT_PULLUP);
  pinMode (SD_CS, INPUT_PULLUP);
  Serial.println("========SDCard Detect.======");
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);
  delay(1000);
  if (!SD.begin(SD_CS)) {
      Serial.println("SDCard MOUNT FAIL");
  } else {
    uint32_t cardSize = SD.cardSize() / (1024 * 1024);
    String str = "SDCard Size: " + String(cardSize) + "MB";
    Serial.println(str);
    FlagSDCardPresent = true;
  }
  Serial.println("===========================");


  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX, false);
  modem_on();
  Serial.print(F("Reset modem: "));
  Serial.println(modem.setPhoneFunctionality(1,1));// CFUN=1,1 full functionality, reset online mode
  delay(200);
  Serial.print(F("Modem Info: "));
  Serial.println(modem.getModemInfo());
  modem.setNetworkMode(38);  // Network Mode LTE
  modem.setPreferredMode(1); // Mode CAT-M
  // Allumage du GPS
  modem.send_AT("+CGPIO=0,48,1,1"); // Alimentation antenne GPS
  opmessage += "turn ON GPS";
  opmessage += modem.enableGPS();
  opmessage += fl;

  
  init_adc_mm();// initialisation tableau pour adc Moyenne Mobile
  ADC.attach_ms(100, adc_read); // lecture des adc toute les 100ms

  Blinker.attach_ms(BlinkerInterval, Blink); // lancement Blinker lent

  /* Lecture configuration file config	 */
  readConfig(); // Lecture de la config
  if (config.magic != Magique) {
    /* verification numero magique si different
    		erreur lecture EEPROM ou carte vierge
    		on charge les valeurs par défaut
    */
    Serial.println(F("Nouvelle Configuration !"));
    config.magic            = Magique;
    config.tracker          = true;
    config.trapide          = 10;      // secondes
    config.tlent            = 15;      // secondes
    config.vtransition      = 2;       // kmh
    config.timeoutWifi      = 10 * 60;
    config.hete             = 2;
    config.hhiver           = 1;
    config.mqttPort         = tempmqttPort;
    config.sendSMS          = false;
    config.logSDcard        = false;
    config.keepAlive        = 300; // 5mn, IMPERATIF pour réduire conso data    
    String temp             = "TPCF_TTX01";
    temp.toCharArray(config.Idchar, 11);
    String tempapn          = "eapn1.net";//"sl2sfr";//"free"
    String tempUser         = "";
    String tempPass         = "";
    tempapn.toCharArray(config.apn, (tempapn.length() + 1));
    tempUser.toCharArray(config.gprsUser, (tempUser.length() + 1));
    tempPass.toCharArray(config.gprsPass, (tempPass.length() + 1));
    tempServer.toCharArray(config.mqttServer, (tempServer.length() + 1));
    tempmqttUserName.toCharArray(config.mqttUserName, (tempmqttUserName.length() + 1));
    tempmqttPass.toCharArray(config.mqttPass, (tempmqttPass.length() + 1));
    copie_Topic();

    sauvConfig();
  }
  PrintConfig();

  strncpy(willTopic,("L/will"),sizeof(willTopic));// topic commun

  Id  = String(config.Idchar);
  Id += fl;

  logmessage();

  ArduinoOTA.setHostname(config.Idchar);
  ArduinoOTA.setPasswordHash(OTApwdhash);
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.print(F("Start updating "));
    Serial.println(type);
  })
  .onEnd([]() {
    Serial.println(F("End"));
    Alarm.delay(1000);
    ESP.restart();
    ResetHard();
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if 			(error == OTA_AUTH_ERROR) 		Serial.println(F("Auth Failed"));
    else if (error == OTA_BEGIN_ERROR) 		Serial.println(F("Begin Failed"));
    else if (error == OTA_CONNECT_ERROR) 	Serial.println(F("Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR) 	Serial.println(F("Receive Failed"));
    else if (error == OTA_END_ERROR) 			Serial.println(F("End Failed"));
  });

  
  OuvrirFichierCalibration(); // ouvre fichier calibration en SPIFFS
  
  Ouvrir_PB();                // ouvre le fichier Phone book

  Serial.print(("Waiting for network..."));
  if (!modem.waitForNetwork()) {
    Serial.println(F(" fail"));
    delay(100);
    // return;
  } else {
    Serial.println(F(" success"));
  }
  if (modem.isNetworkConnected()) {
    Serial.println("Network connected");

    // Demande Operateur connecté
    Serial.print(F("Operateur :")), Serial.println(modem.getOperator());

    ConnectGPRS();

    if (modem.isGprsConnected()) { Serial.println(F("GPRS connected")); }
    IPAddress local = modem.localIP();
    Serial.println("Local IP:" + local.toString());

    Serial.print("Signal quality:"), Serial.println(read_RSSI());
  }

  mqttClient.setBufferSize(384);
  mqttClient.setKeepAlive(config.keepAlive);                // Set Pubsub keep alive interval
  mqttClient.setServer(config.mqttServer, config.mqttPort); // Set the MQTT broker details.
  mqttClient.setCallback(mqttSubscriptionCallback);         // Set the MQTT message handler function.

  // Synchro heure
  readmodemtime();           // Lecture heure modem
  // set modem Localtime
  Serial.println("set CLTS=1:"),Serial.println(modem.send_AT("+CLTS=1"));
  timesstatus();								// Etat synchronisation Heure Sys
  // sync Heure system avec modem (si Heure modem valide)
  // place FlagSetTime=true si Ok, sinon relancer dans la boucle
  set_system_time();
  timesstatus();								// Etat synchronisation Heure Sys
  Serial.print("Syst Time ="),Serial.println(displayTime(0));
  
  loopPrincipale = Alarm.timerRepeat(10, Acquisition); // boucle principale 10s
  Alarm.enable(loopPrincipale);

  first = Alarm.timerOnce(60, once); // premier lancement
  Alarm.enable(first);

  Send = Alarm.timerRepeat(config.tlent, senddata); // send data
  Alarm.disable(Send);
  
  // Svie = Alarm.alarmRepeat(7*3600, SignalVie); // chaque jour 07H00
  // Alarm.enable(Svie);
  
  // Acquisition();
  // reduire consommation eteindre Wifi et BT
  WiFi.mode(WIFI_OFF);
  btStop();

  Check_Online();
}
//---------------------------------------------------------------------------
void loop() {
  static unsigned int t0 = millis(); // timer checkOnline
  if(millis() - t0 > 5000){
    t0 = millis();
    Check_Online();
  }
  // Gestion LED externe
  if(Online){
    // FastBlink
    
  } else {
    // SlowBlink

  }
  recvOneChar(); // Capture reception liaison serie locale
  // receiveBLE();  // Capture reception depuis BT
  
  if(Online){
    mqttClient.loop(); // Call the loop to maintain connection to the server.
  }

  ArduinoOTA.handle();
  Alarm.delay(0);
}
//---------------------------------------------------------------------------
void Acquisition() {
  bool first = true;
  static float lastlon = 0;
  static float lastlat = 0;

  if (!FlagSetTime){ // si mise à l'heure non valide
    StartStopAlarme(false);
    set_system_time();
    StartStopAlarme(true);
  }

  Serial.println(displayTime());

  opmessage = displayTime() + fl;
  opmessage += modem.getOperator();
  opmessage += fl + read_RSSI();
  // IPAddress local = modem.localIP();
  // opmessage += ", Local IP:" + local.toString();
  logmessage();

  static byte lastminute = minute();
  // if(minute() == 0 && minute() != lastminute){ // test
  // lastminute = minute();
  //   opmessage = displayTime() + fl;
  //   opmessage += modem.send_AT("+CPSI?");
  //   Publication();
  // } else {
  //   lastminute = minute();
  // }
  if (CoeffTension[0] == 0 || CoeffTension[1] == 0){// || CoeffTension[2] == 0 || CoeffTension[3] == 0) {
    OuvrirFichierCalibration(); // patch relecture des coeff perdu
  }
  int smsnum = -1;
  if (Online){
    // verification index new SMS en attente(raté en lecture directe)
    // Si SMS reçu , lire en boucle les sms ReadSMS(smsnum)
    int smsnum = modem.newMessageIndex(0); // verifie index arrivée sms, -1 si pas de sms
    opmessage = "Index last SMS = " + String(smsnum);
    logmessage();
  }
  if (smsnum >= 0) {	// index du SMS en attente
    // il faut les traiter
    ReadSMS(smsnum);// Lecture et traitement de tous les SMS en attente
  }
  else if (smsnum < 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
    FlagReset = false;
    ResetHard();					//	redemarrage ESP
  }

  Serial.print("tension proc = "), Serial.print(VBatterieProc / 1000.0, 2), Serial.print("V");
  Serial.print("; tension alim = "), Serial.print(VAlim / 1000.0, 2), Serial.println("V");

  static byte nalaAlim = 0;
  static byte nRetourTension = 0;
  // if (VAlim < 3300) {
  //   if (nalaAlim ++ == 10) {
  //     FlagAlarmeTension = true;
  //     nalaAlim = 0;
  //   }
  // }
  // else if (VAlim > 4500) { // A Finir
  //   nRetourTension ++;
  //   if (nRetourTension == 4) {
  //     FlagAlarmeTension = false;
  //     nRetourTension = 0;
  //   }
  // }
  // else {
  //   if (nalaAlim > 0)nalaAlim --;
  // }
  FlagAlarmeTension = false;

  // if (config.tracker){
    // static byte nalaGprs = 0;
    // if (!modem.isGprsConnected()) {
    //   modem.gprsConnect(config.apn, config.gprsUser, config.gprsPass);
    //   if (nalaGprs ++ == 10) {
    //     FlagAlarmeGprs = true;
    //     nalaGprs = 0;
    //   }
    // } else {
    //   // if (nalaGprs > 0){
    //   //   nalaGprs --;
    //   //   FlagAlarmeGprs = false;
    //   // }
    //   nalaGprs = 0;
    //   FlagAlarmeGprs = false;
    // }
    // static byte nalaMQTT = 0;
    // if (!mqttClient.connected()) {
    //   if (nalaMQTT ++ >= 10) {
    //     FlagAlarmeMQTT = true;
    //     nalaMQTT = 0;
    //   }
    // } else {
    //   // if (nalaMQTT > 0){
    //   //   nalaMQTT --;
    //   //   FlagAlarmeMQTT = false;
    //   // }
    //   nalaMQTT = 0;
    //   FlagAlarmeMQTT = false;
    // }
  // } else {
  //   FlagAlarmeGprs = false;
  //   FlagAlarmeMQTT = false;
  // }
  static byte nalaGps = 0;
  if (config.tracker){
    if (!decodeGPS()) {
      if (nalaGps ++ >= 10) {
        opmessage = "turn OFF GPS";
        modem.disableGPS();
        Alarm.delay(1000);
        opmessage += ", turn ON GPS";
        #ifdef LILYGO_SIM7000G
          modem.send_AT("+CGPIO=0,48,1,1"); // Alimentation antenne GPS
        #endif
        modem.enableGPS();
        Alarm.delay(1000);
        FlagAlarmeGps = true;
        nalaGps = 0;
        logmessage();
      }
    } else {
      if (nalaGps > 0){
        nalaGps --;
      } else {
        FlagAlarmeGps = false;
      }
    }
    gereCadence();
    Serial.print("distance:"), Serial.println(calc_dist(lat, lon, lastlat, lastlon), 5);
    lastlat = lat;
    lastlon = lon;
  }
  envoie_alarme();

  digitalWrite(LED_PIN,LOW);
  Alarm.delay(100);
  digitalWrite(LED_PIN,HIGH);
}
//---------------------------------------------------------------------------
// check if we are Oline
void Check_Online(){
  static int cptRegStatusFault = 0; // compteur defaut network
  bool net_status  = false;
  bool gprs_status = false;
  bool mqtt_status = false;

  net_status = modem.isNetworkConnected();
  if (net_status){
    if (modem.isGprsConnected()){
      gprs_status = true;
    } else {
      if (ConnectGPRS()){
        gprs_status = true;
      }
    }
  }
  if (net_status && gprs_status){
    if (!mqttClient.connected()){ // mqtt pas connecté, mqtt connect et on relance subscribe
      if (mqttConnect()){ // (re)Connect if MQTT client is not connected.
        if (mqttSubscribe(0) == true) {
          opmessage = "Subscribed";
          logmessage();
          mqtt_status = true;
        }
      }
    } else { // si mqqt connecté, suppose subscribe ok
      mqtt_status = true;
    }
  }

  if(net_status && gprs_status && mqtt_status){ // we are Online
    Online = true;
    Change(true);
  } else {
    Online = false;
    lastOnline = false;
    Change(false);
  }

  if (Online && !lastOnline){
    lastOnline = Online;
    if(config.tracker && Accu == 0){
      // Oblige à envoyer position apres un retour réseau
      // si vitesse = 0
      Accu = 255;
      gereCadence();
    }
  }

  opmessage = displayTime() + fl;
  opmessage += "Online,res,gprs,mqtt,nRegFault :" + String(Online) + ":" + String(net_status) + ":" + String(gprs_status) + ":" + String(mqtt_status) + ":" + String(cptRegStatusFault) + fl;
  
  logmessage();

  // Patch Blocage modem
  if(! net_status){
    if(cptRegStatusFault ++ > 24){ // si hors réseau > 24*5s
      cptRegStatusFault = 0;
      NbrResetModem +=1;
      opmessage = "Reset modem suite Reg status fault" + String(cptRegStatusFault);
      logmessage();
      modem.send_AT(F("+CFUN=1,1"));
      delay(10000);
    }
  } else {
    cptRegStatusFault = 0;// reset compteur
  }
}
//---------------------------------------------------------------------------
void once() {
  Serial.println("Premier lancement");
  if(config.tracker){
    Accu = 255; // forcer plusieurs envoie au lancement
    Alarm.write(Send, config.tlent);
    senddata();
  }
}
//---------------------------------------------------------------------------
void senddata() {
  if (decodeGPS() && Online) {    
    char bidon[20];
    sprintf(bidon, "%04d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());

    dataToPublish = String(config.Idchar) +";"+ bidon +";"+ String(lat, 8) +";"+ String(lon, 8)+";"+String(speed, 1) +";"+String(course, 0);
    bool rep = mqttClient.publish(config.sendTopic[0], dataToPublish.c_str(), true); // retain
    opmessage = "publication :";
    opmessage += String(rep);
    opmessage += ";";
    dataToPublish.setCharAt(42,' '); // remplacer ';' par ' ' entre lat et lon, + facile exploitation du log
    opmessage += dataToPublish;
    logmessage();
  }
}
//---------------------------------------------------------------------------
// void Publication(){
//   if(Online){
//     bool rep = mqttClient.publish(config.sendTopic[1], opmessage.c_str(), true);// retain
//     Serial.print("publication :"), Serial.println(rep);
//   }
// }
//---------------------------------------------------------------------------
void envoie_alarme() {
  /* determine si un SMS appartition/disparition Alarme doit etre envoyé */
  bool SendEtat = false;

  if (FlagAlarmeTension != FlagLastAlarmeTension) {
    SendEtat = true;
    FlagLastAlarmeTension = FlagAlarmeTension;
  }
  if (FlagAlarmeGprs != FlagLastAlarmeGprs) {
    SendEtat = true;
    FlagLastAlarmeGprs = FlagAlarmeGprs;
  }
  if (FlagAlarmeMQTT != FlagLastAlarmeMQTT) {
    SendEtat = true;
    FlagLastAlarmeMQTT = FlagAlarmeMQTT;
  }
  if (FlagAlarmeGps != FlagLastAlarmeGps) {
    SendEtat = true;
    FlagLastAlarmeGps = FlagAlarmeGps;
  }
  if (SendEtat) { 							// si envoie Etat demandé
    envoieGroupeMessage(false, true);	 // pasVie, Serveur
    envoieGroupeMessage(false, false); // pasVie, User
    SendEtat = false;						// efface demande
  }
}
//---------------------------------------------------------------------------
void envoieGroupeMessage(bool vie, bool Serveur) {
  generationMessage();
  if(vie){
    message += fl;
    message += F("Reset modem : ");
    message += String(NbrResetModem);
    message += fl;
  }
  if(config.sendSMS){
  // A Finir 
  // for (byte Index = 1; Index < 10; Index++) {		// Balayage des Num Tel Autorisés=1 dans Phone Book
  //   Phone = {"",""};
  //   if(modem.readPhonebookEntry(&Phone, Index)){
  //     Serial.print(Index),Serial.print(","),Serial.println(Phone.number);
  //     if(Phone.number.length() > 0){
  //       if (grp == 1){	// grp = 1 message liste restreinte
  //       // rien
  //       } else {	// grp = 0, message à tous
  //         sendReply(Phone.number , true);
  //       }        
  //     } else {
  //       Index = 10;
  //     }
  //   }
  // }
  }
  // A Finir
  // envoyer vers BLE

  Serial.println(message);
  Envoyer_MQTT(Serveur);
}
//---------------------------------------------------------------------------
void ReadSMS(int index){
  // index du SMS
  // verifier appelant connu si OK copier texte sms dans Rmessage
  // effacer SMS
  // et envoyer traite_sms("SMS")

  String SenderName;
  Sms smsstruct;
  // A Finir 
    if (!modem.readSMS(&smsstruct,index)){
      Serial.print(F("Didn't find SMS message in slot! "));
      Serial.println(index);
    }
    if(! Cherche_N_PB(smsstruct.sendernumber)){
      Serial.println(F("Appelant inconnu"));
      EffaceSMS(index);
      return;
    }
    // Serial.print("Numero:"),Serial.println(smsstruct.sendernumber);
    // Serial.print("name  :"),Serial.println(smsstruct.sendername);
    // Serial.print("message sms:"),Serial.println(smsstruct.message);
    // Serial.print("datetime sms:"),Serial.println(smsstruct.timestamp);
    Rmessage = smsstruct.message;
    EffaceSMS(index);
    traite_sms("SMS");
}
//---------------------------------------------------------------------------
void traite_sms(String Origine) {
  // Origine = Local,BLE,SMS,MQTT
  bool sms = false;
  if(Origine == "SMS") sms = true;

  /* Variables pour mode calibration */
  static int tensionmemo = 0;//	memorisation tension batterie lors de la calibration
  int coef = 0; // coeff temporaire
  static byte P = 0; // Pin entrée a utiliser pour calibration
  static byte M = 0; // Mode calibration 1,2,3,4
  static bool FlagCalibration = false;	// Calibration Tension en cours

  Serial.print("message: "), Serial.print(Rmessage),Serial.print(","),Serial.println(Rmessage.length());
  

  messageId(); // Preparation message reponse
  if (!(Rmessage.indexOf(F("TEL")) == 0 || Rmessage.indexOf(F("tel")) == 0 || Rmessage.indexOf(F("Tel")) == 0
      || Rmessage.indexOf(F("Wifi")) == 0 || Rmessage.indexOf(F("GPRSDATA")) > -1 || Rmessage.indexOf(F("MQTTDATA")) > -1 
      || Rmessage.indexOf(F("MQTTSERVEUR")) > -1)) {
    Rmessage.toUpperCase();    // passe tout en Maj sauf si ci dessus
    Rmessage.replace(" ", ""); // supp tous les espaces
  }

  if (Rmessage.indexOf(F("ID=")) == 0) {			//	Id= nouvel Id
    String temp = Rmessage.substring(3);
    Serial.println(Rmessage),Serial.println(temp);
    if (temp.length() > 0 && temp.length() < 11) {
      Id = "";
      temp.toCharArray(config.Idchar, 11);
      sauvConfig();															// sauvegarde en EEPROM
      Id = String(config.Idchar);
      Id += fl;
    }
    messageId();
    message += F("Nouvel Id");
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("SPEED")) >= 0) { // debug SPEED=xx force envoyé position
    if ((Rmessage.indexOf(char(61))) == 5) {
      int i = Rmessage.substring(6).toInt();
      if (i > 0 && i <= 101) {								//	ok si entre 1 et 100
        speed = i;
      }
    }
    Serial.print("Speed="),Serial.println(speed);
    gereCadence();
    Acquisition();
  }
  else if (Rmessage.indexOf(F("TEL")) == 0
          || Rmessage.indexOf(F("Tel")) == 0
          || Rmessage.indexOf(F("tel")) == 0) { // entrer nouveau num
    byte lastPBline = last_PB(); // recupere le n° de la derniere ligne du PB
    bool newPB = false;
    bool FlagOK = true;
    bool efface = false;
    byte j = 0;
    String newnumero;
    String newnom;
    int indexreplace = 0;
    if (Rmessage.indexOf(char(61)) == 4) {  // TELn= reserver correction/suppression
      int i = Rmessage.substring(3).toInt();// recupere n° de index
      i = i / 1; // important sinon i ne prend pas sa valeur dans les comparaison?
      if (i < 1) FlagOK = false;
      indexreplace = i;// index du PB a remplacer
      j = 5;
      // on efface la ligne sauf la 1 pour toujours garder au moins un numéro
      if ((i != 1) && (i<=lastPBline) &&(Rmessage.indexOf(F("efface")) == 5 || Rmessage.indexOf(F("EFFACE")) == 5 )) {
        efface = true;
        strcpy(PB_list[i] , "");
        if(i < lastPBline){
          // il faut décaler les lignes vers le bas
          for (int ligne = i;ligne<lastPBline;ligne ++){
            strcpy(PB_list[ligne] , PB_list[ligne+1]);
          }
          if(lastPBline < 9){
            strcpy(PB_list[lastPBline] , "");// efface derniere ligne
          }
        }
        Save_PB();
        message += "ligne efface";
        goto fin_tel;
      }
    }
    else if (Rmessage.indexOf(char(61)) == 3) { // TEL= nouveau numero
      j = 4;
      newPB = true;
    }
    else {
      FlagOK = false;
    }
    if (Rmessage.indexOf("+") == j) {			          // debut du num tel +
      if (Rmessage.indexOf(char(44)) == j + 12) {	  // verif si longuer ok
        newnumero = Rmessage.substring(j, j + 12);
        newnom = Rmessage.substring(j + 13, j + 27);// tronque à 14 car
      }
      else {
        FlagOK = false;
      }
    }
    else {
      FlagOK = false;
    }
fin_tel:
    if (!FlagOK) { // erreur de format
      message += F("Cde non reconnue/erreur ?");// non reconnu
      message += fl;
      sendReply(Origine);
    }
    else {
      if (!efface) {
        bool ok = false;
        String bidon = newnumero + ";" + newnom;
        if(newPB){ // Nouvelle ligne
          strcpy(PB_list[lastPBline + 1] , bidon.c_str());
        } else {   // Remplacement ligne
          strcpy(PB_list[indexreplace] , bidon.c_str());
        }
        message += "Nouvelle entree Phone Book :" + fl;
        message += bidon;

      }
      Save_PB();
      sendReply(Origine);
    }
  }
  else if (Rmessage.indexOf("LST") == 0) {	//	Liste des Num Tel
    Read_PB();
    for(byte i = 1;i<10;i++){
      if(strlen(PB_list[i]) > 0){
        // Serial.println(PB_list[i]);
        message += i;
        message += ":";
        message += String(PB_list[i]);
        message += fl;
      }
    }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("ETAT")) == 0 || Rmessage.indexOf(F("ST")) == 0) {			// "ETAT? de l'installation"
    generationMessage();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("SYS")) == 0) { //	SYS? Etat Systeme
    byte n = modem.getRegistrationStatus();
    if (n == 5) {														// Operateur roaming
      message += F("rmg, ");								// roaming
      message += modem.getOperator() + fl;
    }
    else {
      message += modem.getOperator() + fl;  // Operateur
    }
    message += read_RSSI() + fl;
    message += "V SIM7600 = ";
    message	+= String(modem.getBattVoltage());
    message += " mV, ";
    // message += String(modem.getBattPercent()) + "%" + fl;

    message += ("Ver:") + ver + fl;

    message += F("Valim Proc = ");
    message += String(VBatterieProc/1000.0,2) + "V";

    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("TIMERLENT")) == 0) { //	Timer lent
    if ((Rmessage.indexOf(char(61))) == 9) {
      int i = Rmessage.substring(10).toInt();
      if (i > 9 && i <= 3601) {								//	ok si entre 10 et 3600
        config.tlent = i;
        sauvConfig();													// sauvegarde en EEPROM
        Alarm.disable(Send);
        Alarm.write(Send, config.tlent);
      }
    }
    message += F("Timer Lent = ");
    message += String(config.tlent);
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("TIMERRAPIDE")) == 0) { //	Timer rapide
    if ((Rmessage.indexOf(char(61))) == 11) {
      int i = Rmessage.substring(12).toInt();
      if (i > 4 && i <= 3601) {								//	ok si entre 5 et 3600
        config.trapide = i;
        sauvConfig();													// sauvegarde en EEPROM
        Alarm.disable(Send);
        Alarm.write(Send, config.trapide);
      }
    }
    message += F("Timer Rapide = ");
    message += String(config.trapide);
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("VITESSEMINI")) == 0) { //	Vitesse mini
    if ((Rmessage.indexOf(char(61))) == 11) {
      int i = Rmessage.substring(12).toInt();
      if (i > 0 && i <= 21) {								//	ok si entre 1 et 20
        config.vtransition = i;
        sauvConfig();													// sauvegarde en EEPROM
      }
    }
    message += F("Vitesse mini = ");
    message += String(config.vtransition);
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("TRACKER")) == 0) { // Tracker actif
    if (((Rmessage.indexOf(char(61))) == 7) || ((Rmessage.indexOf("O")) == 7)) { // = ou O
      int i = 0;
      if (Rmessage.substring(7) == F("ON")) {
        i = 1;
      } else if (Rmessage.substring(7) == F("OFF")) {
        i = 0;
      } else {
        i = Rmessage.substring(8).toInt();
      }
      if (i == 1) {
        config.tracker = true;
        sauvConfig();																// sauvegarde en EEPROM
        // ConnectGPRS();
        Alarm.disable(Send);
        Alarm.write(Send, config.tlent);
        Accu = 255;
        senddata(); // active la localisation          
      }
      else if (i == 0) {
        config.tracker = false;
        sauvConfig();																// sauvegarde en EEPROM
        // ArretLocalisation();
        Alarm.disable(Send);
        // modem.gprsDisconnect();
      }
    }
    generationMessage();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("TRCKPARAM")) > -1) {
    /* parametres Tracker en sms:
      TRCKPARAM=timerlent:timerrapide:vitessemini
      {"TRCKPARAM":{"timerlent":600,"timerrapide":15,"vitessemini":2}}
    */
    bool erreur = false;
    bool formatsms = false;
    // Serial.print("position X:"),Serial.println(Rmessage.substring(7, 8));
    // Serial.print("position ::"),Serial.println(Rmessage.substring(8, 9));
    if (Rmessage.substring(12, 13) == ":") {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, Rmessage);
      if (err) {
        erreur = true;
      }
      else {
        // Serial.print(F("Deserialization succeeded"));
        JsonObject param = doc["TRCKPARAM"];
        config.tlent = param["TIMERLENT"];
        config.trapide = param["TIMERRAPIDE"];
        config.vtransition = param["VITESSEMINI"];
        sauvConfig();
      }
    }
    else if ((Rmessage.indexOf(char(61))) == 9) { // format sms
      formatsms = true;
      byte cpt = 0;
      byte i = 10;
      do { // compte nombre de : doit etre =2
        i = Rmessage.indexOf(':', i + 1);
        cpt ++;
      } while (i <= Rmessage.length());
      if (cpt - 1 == 2) {
        byte x = Rmessage.indexOf(':');
        byte y = Rmessage.indexOf(':', x + 1);
        int t1 = Rmessage.substring(10, x).toInt();
        int t2 = Rmessage.substring(x + 1, y).toInt();
        int t3 = Rmessage.substring(y + 1).toInt();
        if (t1 > 59 && t1 < 3601 && t2 > 0 && t2 < 1801 && t3 > 0 && t3 < 21) {
          config.tlent = t1;
          config.trapide = t2;
          config.vtransition = t3;
          sauvConfig();
        }
        else {
          erreur = true;
        }
      }
      else {
        erreur = true;
      }
    }
    if (!erreur) {
      if (formatsms) {
        message += F("Tracker parametres\n");
        message += F("timerlent:timerrapide:vitessemini\n");
        message += String(config.tlent) + ":" + String(config.trapide) + ":" + String(config.vtransition);
      }
      else {
        // calculer taille https://arduinojson.org/v6/assistant/
        JsonDocument doc;
        JsonObject param = doc["TRCKPARAM"].to<JsonObject>();
        param["timerlent"] = config.tlent;
        param["timerrapide"] = config.trapide;
        param["vitessemini"] = config.vtransition;
        Sbidon="";
        doc.shrinkToFit();  // optional
        serializeJson(doc, Sbidon);
        // serializeJson(doc, Serial);
        message += Sbidon;
      }
    }
    else {
      message += "erreur format";
    }
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("COEFF")) == 0) {//Lecture/ecriture des coeff
    // COEFF=xxxx,xxxx,xxxx,xxxx
    if(Rmessage.indexOf(char(61)) == 5){ // =
      Sbidon = Rmessage.substring(6, Rmessage.length());
      Serial.println(Sbidon);
      int tempo[4] = {0,0,0,0};
      byte p1 = 0;
      byte p2 = 0;
      bool flag = true;
      for(int i = 0; i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])); i++){          
        // printf("i=%d,p1=%d,p2=%d\n",i,p1,p2);
        p2 = Sbidon.indexOf(char(44), p1 + 1); // ,
        tempo[i] = Sbidon.substring(p1,p2).toInt();
        if(tempo[i] < 0) flag = false;
        if(i!=3 && p2 == 255) flag = false;
        p1 = p2 + 1;          
        // printf("i=%d,p1=%d,p2=%d\n",i,p1,p2);
      }
      if (flag){ // format OK
        for(int i = 0; i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])); i++){
          CoeffTension[i] = tempo[i];
        }
        Recordcalib(); // enregistre en SPIFFS
      }
    }
    message += "Coeff calibration:" + fl;
    for(int i = 0; i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])); i++){
      message += String(CoeffTension[i]);
      if(i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])- 1) ) message += ",";
    }
    Serial.println(message);
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("MQTTDATA") > -1) {
    // Parametres MQTTDATA=Serveur:User:Pass:Topic:port
    bool erreur = false;
    bool formatsms = false;
    if (Rmessage.indexOf(":") == 11) { // format json
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, Rmessage);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject mqttdata = doc["MQTTDATA"];
        strncpy(config.mqttServer,   mqttdata["serveur"], 26);
        strncpy(config.mqttUserName, mqttdata["user"],    11);
        strncpy(config.mqttPass,     mqttdata["pass"],    16);
        // strncpy(config.writeTopic,   mqttdata["topic"],   16);
        config.mqttPort            = mqttdata["port"];
        sauvConfig();													// sauvegarde en EEPROM
      }
    }
    else if ((Rmessage.indexOf(char(61))) == 8) { // format sms
      formatsms = true;
      byte w = Rmessage.indexOf(":");
      byte x = Rmessage.indexOf(":", w + 1);
      byte y = Rmessage.indexOf(":", x + 1);
      byte z = Rmessage.indexOf(":", y + 1);
      byte zz = Rmessage.length();
      // Serial.printf("%d:%d:%d:%d\n",w,x,y,z);
      // Serial.printf("%d:%d:%d:%d:%d\n",w-9,x-w-1,y-x-1,z-y-1,zz-z-1);
      if (Rmessage.substring(z + 1, zz).toInt() > 0) { // Port > 0
        if ((w - 9) < 25 && (x - w - 1) < 11 && (y - x - 1) < 16 && (z - y - 1) < 16) {
          Sbidon = Rmessage.substring(9, w);
          Sbidon.toCharArray(config.mqttServer, (Sbidon.length() + 1));
          Sbidon = Rmessage.substring(w + 1, x);
          Sbidon.toCharArray(config.mqttUserName, (Sbidon.length() + 1));
          Sbidon = Rmessage.substring(x + 1, y);
          Sbidon.toCharArray(config.mqttPass, (Sbidon.length() + 1));
          Sbidon = Rmessage.substring(y + 1, z);
          // Sbidon.toCharArray(config.writeTopic, (Sbidon.length() + 1));
          config.mqttPort = Rmessage.substring(z + 1, zz).toInt();
          sauvConfig();													// sauvegarde en EEPROM
        }
        else {
          erreur = true;
        }
      } else {
        erreur = true;
      }
    }
    if (!erreur) {
      if (formatsms) {
        message += "Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant";
        message += fl;
        message += F("Parametres MQTT :");
        message += fl;
        message += "Serveur:" + String(config.mqttServer) + fl;
        message += "User:"    + String(config.mqttUserName) + fl;
        message += "Pass:"    + String(config.mqttPass) + fl;
        // message += "Topic:"   + String(config.writeTopic) + fl;
        message += "Port:"    + String(config.mqttPort) + fl;
      }
      else {
        JsonDocument doc;
        JsonObject MQTTDATA = doc["MQTTDATA"].to<JsonObject>();
        MQTTDATA["serveur"] = config.mqttServer;
        MQTTDATA["user"]    = config.mqttUserName;
        MQTTDATA["pass"]    = config.mqttPass;
        // MQTTDATA["topic"]   = config.writeTopic;
        MQTTDATA["port"]    = config.mqttPort;
        Sbidon = "";
        doc.shrinkToFit();  // optional
        serializeJson(doc, Sbidon);
        message += Sbidon;
        message += fl;
      }
    }
    else {
      message += "Erreur format";
      message += fl;
    }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("TOPIC") > -1) { // MQTT Topic
    // Parametres TOPIC="sendTopic":"sendtopic0,sendtopic1","recvTopic":"recvtopic0,recvtopic1"
    //"{"TOPIC":{"sendTopic0": "sendtopic0", "sendTopic1": "sendtopic1","recvTopic0":"recvtopic0","recvTopic1":"recvtopic1"}}"
    
    bool erreur = false;
    if (Rmessage.indexOf(":") == 8) { // format json
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, Rmessage);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject TOPIC = doc["TOPIC"];
        strncpy(config.sendTopic[0],TOPIC["sendTopic0"],13);
        strncpy(config.sendTopic[1],TOPIC["sendTopic1"],13);
        strncpy(config.recvTopic[0],TOPIC["recvTopic0"],13);
        strncpy(config.recvTopic[1],TOPIC["recvTopic1"],13);
        sauvConfig();
        copie_Topic();
      }
    }
    if (!erreur){
      JsonDocument doc;
      JsonObject TOPIC = doc["TOPIC"].to<JsonObject>();
      
      TOPIC["sendTopic0"] = config.sendTopic[0];
      TOPIC["sendTopic1"] = config.sendTopic[1];
      TOPIC["recvTopic0"] = config.recvTopic[0];
      TOPIC["recvTopic1"] = config.recvTopic[1];

      Sbidon = "";
      serializeJson(doc, Sbidon);
      message += Sbidon;
      message += fl;
    }
    else {
      message += "Erreur format";
      message += fl;
    }
    sendReply(Origine);
  }  
  else if (Rmessage.indexOf("KEEPALIVE") == 0) { // Keep alive interval
    if (Rmessage.indexOf(char(61)) == 9) {
      uint16_t rep = Rmessage.substring(8).toInt();
      if (rep > 10 && rep < 65535){
        config.keepAlive = rep;
        sauvConfig();
        FlagReset = true;
        message += "sera applique apres redemarrage dans 10s";
        message += fl;
      }
    }
    message += "KeepAlive (s)=";
    message += String(config.keepAlive);
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("MQTTSERVEUR") == 0) { // Serveur MQTT
    // case sensitive
    // MQTTSERVEUR=abcd.org
    if (Rmessage.indexOf(char(61)) == 11) {
      Sbidon = Rmessage.substring(12);
      Serial.print("mqttserveur:"),Serial.print(Sbidon);
      Serial.print(" ,"), Serial.println(Sbidon.length());
      Sbidon.toCharArray(config.mqttServer, (Sbidon.length() + 1));
      sauvConfig();
    }
    message += F("MQTTserveur =");
    message += String(config.mqttServer);
    message += F("\n au prochain demarrage");
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("GPRSDATA")) > -1) {
    // Parametres GPRSDATA = "APN":"user":"pass"
    // GPRSDATA="sl2sfr":"":""
    // {"GPRSDATA":{"apn":"sl2sfr","user":"","pass":""}}
    bool erreur = false;
    bool formatsms = false;
    if (Rmessage.indexOf(":") == 11) { // format json
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, Rmessage);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject gprsdata = doc["GPRSDATA"];
        strncpy(config.apn, gprsdata["apn"], 11);
        strncpy(config.gprsUser, gprsdata["user"], 11);
        strncpy(config.gprsPass, gprsdata["pass"], 11);
        // Serial.print("apn length:"),Serial.println(strlen(gprsdata["apn"]));
        // Serial.print("apn:"),Serial.println(config.apn);
        // Serial.print("user:"),Serial.println(config.gprsUser);
        // Serial.print("pass:"),Serial.println(config.gprsPass);
        sauvConfig();													// sauvegarde en EEPROM
      }
    }
    else if ((Rmessage.indexOf(char(61))) == 8) { // format sms
      formatsms = true;
      byte cpt = 0;
      byte i = 9;
      do { // compte nombre de " doit etre =6
        i = Rmessage.indexOf('"', i + 1);
        cpt ++;
      } while (i <= Rmessage.length());
      Serial.print("nombre de \" :"), Serial.println(cpt);
      if (cpt == 6) {
        byte x = Rmessage.indexOf(':');
        byte y = Rmessage.indexOf(':', x + 1);
        byte z = Rmessage.lastIndexOf('"');
        // Serial.printf("%d:%d:%d\n",x,y,z);
        // Serial.printf("%d:%d:%d\n", x -1 - 10, y-1 - x-1-1, z - y-1-1);
        if ((x - 11) < 11 && (y - x - 3) < 11 && (z - y - 2) < 11) { // verification longueur des variables
          Sbidon = Rmessage.substring(10, x - 1);
          Sbidon.toCharArray(config.apn, (Sbidon.length() + 1));
          Sbidon = Rmessage.substring(x + 1 + 1 , y - 1);
          Sbidon.toCharArray(config.gprsUser, (Sbidon.length() + 1));
          Sbidon = Rmessage.substring(y + 1 + 1, z);
          Sbidon.toCharArray(config.gprsPass, (Sbidon.length() + 1));

          // Serial.print("apn:"),Serial.println(config.apn);
          // Serial.print("user:"),Serial.println(config.gprsUser);
          // Serial.print("pass:"),Serial.println(config.gprsPass);

          sauvConfig();													// sauvegarde en EEPROM
        }
        else {
          erreur = true;
        }
      }
      else {
        erreur = true;
      }
    }
    if (!erreur) {
      if (formatsms) {
        message += "Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant";
        message += fl;
        message += F("Parametres GPRS \"apn\":\"user\":\"pass\"");
        message += fl + "\"";
        message += String(config.apn);
        message += "\":\"";
        message += String(config.gprsUser);
        message += "\":\"";
        message += String(config.gprsPass);
        message += "\"" + fl;
      }
      else {
        JsonDocument doc;
        JsonObject gprsdata = doc["GPRSDATA"].to<JsonObject>();
        gprsdata["apn"]  = config.apn;
        gprsdata["user"] = config.gprsUser;
        gprsdata["pass"] = config.gprsPass;
        Sbidon = "";
        doc.shrinkToFit();  // optional
        serializeJson(doc, Sbidon);
        message += Sbidon;
        message += fl;
      }
    }
    else {
      message += "Erreur format";
      message += fl;
    }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("IMEI")) > -1) {
    message += F("IMEI = ");
    message += modem.getIMEI();
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("TIMEOUTWIFI")) > -1) { // Parametre Arret Wifi
    if (Rmessage.indexOf(char(61)) == 11) {
      int n = Rmessage.substring(12, Rmessage.length()).toInt();
      if (n > 9 && n < 3601) {
        config.timeoutWifi = n;
        sauvConfig();														// sauvegarde en EEPROM
      }
    }
    message += F("TimeOut Wifi (s) = ");
    message += config.timeoutWifi;
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("Wifi")) == 0) { // demande connexion Wifi
    char ssid[20];
    char pwd[20];
    byte pos1 = Rmessage.indexOf(char(44));//","
    byte pos2 = Rmessage.indexOf(char(44), pos1 + 1);
    Sbidon = Rmessage.substring(pos1 + 1, pos2);
    Sbidon.toCharArray(ssid, Sbidon.length() + 1);
    Sbidon  = Rmessage.substring(pos2 + 1, Rmessage.length());
    Sbidon.toCharArray(pwd, Sbidon.length() + 1);
    Serial.print("ssid:"), Serial.println(ssid);
    ConnexionWifi(ssid, pwd, Origine); // reponse sera généré par routine
  }
  else if (Rmessage.indexOf(F("WIFIOFF")) > -1) { // Arret Wifi
    message += F("Wifi off");
    message += fl;
    sendReply(Origine);
    WifiOff();
  }
  else if (Rmessage.indexOf(F("UPDATE")) == 0) { // lancement SW Update
    // A Finir
    // ConnexionServeur(smsstruct.sendernumber, sms, index);
    // sms de reponse sera envoyé par routine
  }
  else if (Rmessage.indexOf(F("MAJHEURE")) == 0) {	//	forcer mise a l'heure V2-19
    // A Finir
    MajHeure(true);
    // if (Origine == "SMS"){// || Origine == "MQTT" || Origine == "BLE") {
    //   String mytime = smsstruct.timestamp.substring(0, 20);
    //   Serial.print(F("heure du sms:")),Serial.println(mytime);
    //   String _temp = F("+CCLK=\"");
    //   _temp += mytime + "\"\r\n";
    //   Serial.print(_temp);
    //   modem.send_AT(_temp);
    //   Alarm.delay(100);
    //   MajHeure(true);			// mise a l'heure forcée
    // }
    // else {
    //   message += F("pas de mise à l'heure en local");
    // }
    
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("POSITION")) == 0) {	// demande position
    // on lance la demande au GPS
    if (decodeGPS()) {
      //http://maps.google.fr/maps?f=q&hl=fr&q=42.8089900,2.2614000
      message += F("http://maps.google.fr/maps?f=q&hl=fr&q=");
      message += String(lat, 6);
      message += F(",");
      message += String(lon, 6);
      message += fl;
      message += F("Vitesse = ");
      message += String(speed, 1);
      message += F("km/h");
      message += fl;
      message += F("Dir = ");
      message += String(course, 0);
    }
    else { // si FIX GPS KO
      message += F("GPS pas verrouille !");
    }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("RST") == 0) {               // demande RESET
    message += F("Le systeme va etre relance");  // apres envoie du SMS!
    message += fl;
    FlagReset = true;                            // reset prochaine boucle
    sendReply(Origine);
  }
  else if (!sms && Rmessage.indexOf(F("CALIBRATION=")) == 0) {
    /* Uniquement =.2 pour cette application */

    /* 	Mode calibration mesure tension
        Seulement en mode serie local
        recoit message "CALIBRATION=.X"
        entrer mode calibration
        Selection de la tension à calibrer X
        X = 1 Valim : PinAlim : CoeffTension1
        X = 2 VProc : PinBattProc : CoeffTension2
        X = 3 NC, VUSB : PinBattUSB : CoeffTension3
        X = 4 NC, Tension24 : Pin24V : CoeffTension4
        effectue mesure tension avec CoeffTensionDefaut retourne et stock resultat
        recoit message "CALIBRATION=1250" mesure réelle en V*100
        calcul nouveau coeff = mesure reelle/resultat stocké * CoeffTensionDefaut
        applique nouveau coeff
        stock en EEPROM
        sort du mode calibration

        variables
        FlagCalibration true cal en cours, false par defaut
        Static P pin d'entrée
        static int tensionmemo memorisation de la premiere tension mesurée en calibration
        int CoeffTension = CoeffTensionDefaut 7000 par défaut
    */
    String Sbidon = Rmessage.substring(12, 16); // texte apres =
    //Serial.print(F("Sbidon=")),Serial.print(Sbidon),Serial.print(char(44)),Serial.println(Sbidon.length());
    long tension = 0;
    if (Sbidon.substring(0, 1) == "." && Sbidon.length() > 1) { // debut mode cal
      if (Sbidon.substring(1, 2) == "1" ) {
        M = 1;
        P = PinAlim;
        coef = CoeffTension[0];
      }
      if (Sbidon.substring(1, 2) == "2" ) {
        M = 2;
        P = PinBattProc;
        coef = CoeffTension[1];
      }
      // if (Sbidon.substring(1, 2) == "3" ) {
      // M = 3;
      // P = PinBattUSB;
      // coef = CoeffTension[2];
      // }
      // if (Sbidon.substring(1, 2) == "4" ) {
      // Allumage(); // Allumage
      // for (int i = 0; i < 5 ; i++) {
      // read_adc(PinBattSol, PinBattProc, PinBattUSB, Pin24V, PinLum); // lecture des adc
      // Alarm.delay(100);
      // }
      // M = 4;
      // P = Pin24V;
      // coef = CoeffTension[3];
      // }
      Serial.print("mode = "), Serial.print(M), Serial.println(Sbidon.substring(1, 2));
      FlagCalibration = true;

      coef = CoeffTensionDefaut;
      tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
      // Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
      tensionmemo = tension;
    }
    else if (FlagCalibration && Sbidon.substring(0, 4).toInt() > 0 && Sbidon.substring(0, 4).toInt() <= 10000) {
      // si Calibration en cours et valeur entre 0 et 5000
      Serial.println(Sbidon.substring(0, 4));
      /* calcul nouveau coeff */
      coef = Sbidon.substring(0, 4).toFloat() / float(tensionmemo) * CoeffTensionDefaut;
      // Serial.print("Coeff Tension = "),Serial.println(coef);
      tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
      CoeffTension[M - 1] = coef;
      FlagCalibration = false;
      Recordcalib();														// sauvegarde en SPIFFS

      // if (M == 4) {
      // Extinction(); // eteindre
      // }
    }
    else {
      message += F("message non reconnu");
      message += fl;
      FlagCalibration = false;
    }
    message += F("Mode Calib Tension ");
    message += String(M) + fl;
    message += F("TensionMesuree = ");
    message += tension;
    message += fl;
    message += F("Coeff Tension = ");
    message += coef;
    // if (M == 1) {
    // message += fl;
    // message += F("Batterie = ");
    // message += String(BattPBpct(tension, 6));
    // message += "%";
    // }
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("SETNETWORKMODE")) >= 0) {// Set Prefered network Mode
    if(Rmessage.indexOf(char(61)) == 14){
      int mode = Rmessage.substring(15).toInt();
      if(mode == 2 || mode == 13 || mode == 38 || mode == 51){
        modem.setNetworkMode(mode);
        delay(1000);
      }
    }
    message += String(modem.send_AT(F("+CNMP?")));
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("SENDAT")) == 0){
    // envoie commande AT au SIM7600
    // ne pas mettre AT au debut, +CPBR=1
    // attention DANGEREUX pas de verification!
    if (Rmessage.indexOf(char(61)) == 6) {
      String CdeAT = Rmessage.substring(7, Rmessage.length());
      message += String(modem.send_AT(CdeAT));
      sendReply(Origine);
    }
  }
  else if (Rmessage.indexOf(F("MODEMINFO")) == 0){
    // Get Modem Info
    message += modem.getModemInfo();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("CPTRESETMODEM")) == 0){
    // Demande nombre de reset modem
    message += F("Compteur reset Modem : ");
    message += String(NbrResetModem);
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("TIMESAVING")) == 0) {
    // TIMESAVING=2:1  +-UTC ete:hiver
    if ((Rmessage.indexOf(char(61))) == 10) {
      int x = Rmessage.indexOf(":");
      int i = atoi(Rmessage.substring(11, x).c_str());
      int j = atoi(Rmessage.substring(x + 1).c_str());
      if (i < 13 && i > -13 && j < 13 && j > -13) {
        config.hete = i;
        config.hhiver = j;
        sauvConfig();
        MajHeure(false);
      }
    }
    message += F("TIMESAVING UTC e:h= ");
    message += String(config.hete) + ":" + String(config.hhiver) + fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("LOGSDCARD")) == 0) { // log sur carte SD seulement LILYGO_SIM7000G
    #ifdef LILYGO_SIM7000G
      if(FlagSDCardPresent){
        if (((Rmessage.indexOf(char(61))) == 9) || ((Rmessage.indexOf("O")) == 9)) { // = ou O
          int i = 0;
          if (Rmessage.substring(9) == F("ON")) {
            i = 1;
          } else if (Rmessage.substring(9) == F("OFF")) {
            i = 0;
          } else {
            i = Rmessage.substring(10).toInt();
          }
          
          if (i == 1) {
            config.logSDcard = true;            
          } else if (i == 0) {
            config.logSDcard = false;            
          }
          sauvConfig();
        }
        message += "log SD card ";
        if(config.logSDcard){
          message += "ON";
        } else {        
          message += "OFF";
        }

      } else {
        message += "pas de carte SD";
      }

    #else
      message += "pas de carte SD sur ce tracker";
    #endif
    sendReply(Origine);
  }
  else {
    message += F("message non reconnu");
    sendReply(Origine);
  }
}
//---------------------------------------------------------------------------
void gereCadence() {
  // change cadence envoie GPS data selon la vitesse
  static bool lastroule = false;
  if (int(speed) > config.vtransition) {
    if (Accu < 255 - int(speed)) {
      Accu += speed;
    }
  } else {
    if (Accu >= 10) {
      Accu = Accu - 10;
      if (Accu < 10)Accu = 0;
    }
  }
  if (Accu == 0 && lastroule == true) {
    lastroule = false;
    Alarm.disable(Send);
    Alarm.write(Send, config.tlent);
    senddata();
    Serial.print("cadence  = "), Serial.println(config.tlent);
  }
  else if (Accu > 0 && lastroule == false) {
    // Accu += 20;// bugg? augmente au demarrage, si speed faible evite Accu de retomber trop vite
    lastroule = true;
    Alarm.disable(Send);
    Alarm.write(Send, config.trapide);
    senddata();
    Serial.print("cadence  = "), Serial.println(config.trapide);
  }
  Serial.print("speed = "), Serial.print(speed);
  Serial.print(", Accu  = "), Serial.println(Accu);
}
//---------------------------------------------------------------------------
bool decodeGPS() {
  static float lastcourse = 0;
  float alt = 0;
  int vsat, usat;
  bool fix = false;
  fix = modem.getGPS(&lat, &lon, &speed, &alt, &course, &vsat, &usat);
  // Serial.print("fix=");
  // Serial.println(fix);
  // Serial.print("lat=");
  // Serial.println(lat,6);
  // Serial.print("lon=");
  // Serial.println(lon,6);
  // Serial.print("alt=");
  // Serial.println(alt);
  // Serial.print("speed=");
  // Serial.println(speed);    
  // Serial.print("course=");
  // Serial.println(course);

  if (speed < config.vtransition){ // evité course fantaisiste à l'arret, recopie course precedent
    course = lastcourse;
  }
  lastcourse = course;
  return fix;
}
//---------------------------------------------------------------------------
// Reception message subscription
void mqttSubscriptionCallback( char* topic, byte* payload, unsigned int mesLength ) {
  /* 6) Use the mqttSubscriptionCallback function to handle incoming MQTT messages.
    The program runs smoother if the main loop performs the processing steps instead of the callback.
    In this function, use flags to cause changes in the main loop. */
  /**
    Process messages received from subscribed channel via MQTT broker.
      topic - Subscription topic for message.
      payload - Field to subscribe to. Value 0 means subscribe to all fields.
      mesLength - Message length.
  */
  // variable stockage temporaire
  static char temptopic[12];
  static String tempmessage = "";

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(" len:");
  Serial.print(mesLength);
  Serial.print(". Message: ");
  
  Sbidon = "";
  
  for (int i = 0; i < mesLength; i++) {
    Serial.print((char)payload[i]);
    Sbidon += (char)payload[i];
  }
  Serial.println();

  /* si len>0, flagRcvMQTT = true, le traitement des commandes bloquantes
    NONCIRCULE, "Wifi,SSID,PW"
    ne seront pas executées,
    ne le seront qu'au retour de flagRcvMQTT = false
    garantie que le message à bien été effacé du serveur
    cela évitera un bouclage sur ce message
    ATTENTION
    entre temps les variables Rmessage et Origine ne doivent pas etres altérées!
    */
  if(Sbidon.length() > 0){
    flagRcvMQTT = true;
    // sauvegarde topic et message
    tempmessage = Sbidon;
    // temptopic   = String(topic);
    strcpy(temptopic,topic);
    Rmessage    = Sbidon;
    // on efface le topic sur le serveur
    if(strcmp(topic,config.recvTopic[0]) == 0){ // Serveur
      Serial.println(mqttClient.publish(config.recvTopic[0],"")); // efface topic sur serveur
    } else if(strcmp(topic,config.recvTopic[1]) == 0){ // User
      Serial.println(mqttClient.publish(config.recvTopic[1],"")); // efface topic sur serveur    
    }
    Serial.println(Rmessage);
    if(flagRcvMQTT){
      // message bloquant sera traité apres retour message len=0.
      Serial.println("message sera traite apres reception message len=0");
      return;
    } else {
      flagRcvMQTT = false;
      Serial.println("message traite maintenant");
      // message non bloquant, on traite de suite
      if(strcmp(temptopic,config.recvTopic[0]) == 0){ // Serveur
        Serial.println("message from serveur");
        traite_sms("MQTTS");
      } else if(strcmp(temptopic,config.recvTopic[1]) == 0){ // User
        Serial.println("message from user");
        traite_sms("MQTTU");
      }
    }
  } else if(Sbidon.length() == 0){
    Serial.println("len = 0");
    if (flagRcvMQTT){
      flagRcvMQTT = false;
      Rmessage = tempmessage;
      // on traite maintenant
      Serial.println("on traite maintenant");
      if(strcmp(temptopic,config.recvTopic[0]) == 0){ // Serveur
        traite_sms("MQTTS");
      } else if(strcmp(temptopic,config.recvTopic[1]) == 0){ // User
        traite_sms("MQTTU");
      }
    }
  }
}
//---------------------------------------------------------------------------
bool mqttConnect() {
  // 7) Use the MQTTConnect function to set up and maintain a connection to the MQTT.

  // if (modem.isGprsConnected()) {
    // Loop until connected.
    while ( !mqttClient.connected()) {
      // Connect to the MQTT broker.
      Serial.print("Attempting MQTT connection...");
      if ( mqttClient.connect(config.Idchar, config.mqttUserName, config.mqttPass,config.sendTopic[0],1,true,config.Idchar,true)) {
        Serial.println( "Connected with Client ID:  " + String(config.Idchar) + " User " + String(config.mqttUserName) + " Pwd " + String(config.mqttPass));
        return true;
      } else {
        Serial.print( "failed, rc = " );
        // See https://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
        Serial.print( mqttClient.state() );
        Serial.println( " Will try again in 5 seconds" );
        // Alarm.delay(5000);
        // break;
        return false;
      }
    }
  // }
}//---------------------------------------------------------------------------
// Subscription MQTT, qos=1
bool mqttSubscribe(bool unsubSub) {
  byte rep = 1;
  // unsubSub = 0 subscribe, = 1 unsubscribe
  if (unsubSub == 0) {
    for(byte i = 0; i<2;i++){
      if (mqttClient.subscribe( config.recvTopic[i] , 1 )){ // Subscribe
        rep *= rep;
      } else {
        rep *= 0;
      }      
      delay(200);
    }
    return rep;
  } else {
    for(byte i = 0; i<2;i++){
      if (mqttClient.unsubscribe(config.recvTopic[i])){ // Unsubscribe
        rep *= rep;
      } else {
        rep *= 0;
      }
      delay(200);
    }
    return rep;
  }
}
//---------------------------------------------------------------------------
void generationMessage() {
  /* Generation du message etat/alarme général */

  messageId();
  if (FlagAlarmeTension || FlagAlarmeGprs || FlagAlarmeMQTT || FlagAlarmeGps) {
    message += F("--KO--------KO--");
  }
  else {
    message += F("-------OK-------");
  }
  message += fl;

  if(config.tracker){
    message += "Tracker ON" + fl;
    }else{
    message += "Tracker OFF" + fl;
  }
  if (Online) {
    message += "Online OK" + fl;
  } else {
    message += "Online KO" + fl;
  }
  // if (!FlagAlarmeGprs) {
  //   message += "GPRS OK" + fl;
  // } else {
  //   message += "GPRS KO" + fl;
  // }

  // if (!FlagAlarmeMQTT) {
  //   message += "MQTT OK" + fl;
  // } else {
  //   message += "MQTT KO" + fl;
  // }
  
  if (!FlagAlarmeGps) {
    message += "GPS OK" + fl;
  } else {
    message += "GPS KO" + fl;
  }

  message += F("Alimentation : ");				//"Alarme Batterie : "
  if (FlagAlarmeTension) {
    message += F("Alarme, ");
    // message += fl;// V2-15
  }
  else {
    message += F("OK, ");
    // message += fl;// V2-15
  }
  // message += String(VAlim/1000.0, 2) + "V";
  message += String(modem.getBattVoltage());
  // message += fl;
  // message += "Valim proc = ";
  // message += String(VBatterieProc/1000.0, 2) + "V";
  // message += fl;
}
//---------------------------------------------------------------------------
void init_adc_mm(void) {
  //initialisation des tableaux
  /* valeur par defaut facultative,
  	permet d'avoir une moyenne proche
  	du resulat plus rapidement
  	val defaut = valdefaut*nSample */
  unsigned int ini_adc1 = 0;// val defaut adc 1
  unsigned int ini_adc2 = 0;// val defaut adc 2
  unsigned int ini_adc3 = 0;// val defaut adc 3
  unsigned int ini_adc4 = 0;// val defaut adc 4
  unsigned int ini_adc5 = 0;// val defaut adc 5
  for (int plus_ancien = 0; plus_ancien < nSample; plus_ancien++) {
    adc_hist[0][plus_ancien] = ini_adc1;
    adc_hist[1][plus_ancien] = ini_adc2;
    adc_hist[2][plus_ancien] = ini_adc3;
    adc_hist[3][plus_ancien] = ini_adc4;
    adc_hist[4][plus_ancien] = ini_adc5;
  }
  //on commencera à stocker à cet offset
  adc_mm[0] = ini_adc1;
  adc_mm[1] = ini_adc2;
  adc_mm[2] = ini_adc3;
  adc_mm[3] = ini_adc4;
  adc_mm[4] = ini_adc5;
}
//---------------------------------------------------------------------------
void adc_read() {
  read_adc(PinAlim, PinBattProc); // lecture des adc
  
  VAlim         = map(adc_mm[0] / nSample, 0, 4095, 0, CoeffTension[0]);
  VBatterieProc = map(adc_mm[1] / nSample, 0, 4095, 0, CoeffTension[1]);
}
//---------------------------------------------------------------------------
void read_adc(int pin1, int pin2) {
  // http://www.f4grx.net/algo-comment-calculer-une-moyenne-glissante-sur-un-microcontroleur-a-faibles-ressources/
  static int plus_ancien = 0;
  //acquisition
  int sample[5];
  for (byte i = 0; i < 5; i++) {
    if (i == 0)sample[i] = moyenneAnalogique(pin1);
    if (i == 1)sample[i] = moyenneAnalogique(pin2);
    // if (i == 2)sample[i] = moyenneAnalogique(pin3);
    // if (i == 3)sample[i] = moyenneAnalogique(pin4);
    // if (i == 4)sample[i] = moyenneAnalogique(pin5);

    //calcul MoyenneMobile
    adc_mm[i] = adc_mm[i] + sample[i] - adc_hist[i][plus_ancien];

    //cette plus ancienne valeur n'est plus utile, on y stocke la plus récente
    adc_hist[i][plus_ancien] = sample[i];
  }
  plus_ancien ++;
  if (plus_ancien == nSample) { //gestion du buffer circulaire
    plus_ancien = 0;
  }
}
//---------------------------------------------------------------------------
int moyenneAnalogique(int Pin) {	// calcul moyenne 10 mesures consécutives
  int moyenne = 0;
  for (int j = 0; j < 10; j++) {
    // Alarm.delay(1);
    moyenne += analogRead(Pin);
  }
  moyenne /= 10;
  return moyenne;
}
//---------------------------------------------------------------------------
void EffaceSMS(int s) {
  bool err;
  byte n = 0;
  do {
    err = modem.deleteSmsMessage(s);
    n ++;
    Serial.print(F("resultat del Sms "));	Serial.println(err);
    if (n > 10) { // on efface tous si echec
      err = modem.deleteSmsMessage(0,4);// au cas ou, efface tous les SMS envoyé/reçu
      Serial.print(F("resultat delall Sms "));	Serial.println(err);
      break;
    }
  } while (!err);
}
//---------------------------------------------------------------------------
void ConnexionWifi(char* ssid, char* pwd, String Origine) {
  mqttClient.disconnect();
  modem.gprsDisconnect();
  Alarm.disable(Send);
  Alarm.disable(loopPrincipale);
  setCpuFrequencyMhz(240);
  messageId();
  Serial.print(F("connexion Wifi:")), Serial.print(ssid), Serial.print(char(44)), Serial.println(pwd);
  String ip;
  WiFi.begin(ssid, pwd);
  // WiFi.mode(WIFI_STA);
  byte timeout = 0;
  bool error = false;

  while (WiFi.status() != WL_CONNECTED) {
    Alarm.delay(1000);
    Serial.print(".");
    timeout ++;
    if (timeout > 60) {
      error = true;
      break;
    }
  }
  if (!error) {
    Serial.println();
    Serial.println(F("WiFi connected"));
    Serial.print(F("IP address: "));
    ip = WiFi.localIP().toString();
    Serial.println(ip);
    ArduinoOTA.begin();

    server_wifi.on("/",         HomePage);
    server_wifi.on("/download", File_Download);
    server_wifi.on("/upload",   File_Upload);
    server_wifi.on("/fupload",  HTTP_POST, []() {
      server_wifi.send(200);
    }, handleFileUpload);
    server_wifi.on("/delete",   File_Delete);
    server_wifi.on("/dir",      SPIFFS_dir);
    // server_wifi.on("/cal",      CalendarPage);
    server_wifi.on("/Tel_list", Tel_listPage);
    // server_wifi.on("/LumLUT",   LumLUTPage);
    server_wifi.on("/timeremaining", handleTime); // renvoie temps restant sur demande
    server_wifi.on("/datetime", handleDateTime); // renvoie Date et Heure
    server_wifi.on("/wifioff",  WifiOff);
    ///////////////////////////// End of Request commands
    server_wifi.begin();
    Serial.println(F("HTTP server started"));

    message += F("Connexion Wifi : ");
    message += fl;
    message += String(ip);
    message += fl;
    message += String(WiFi.RSSI());
    message += F(" dBm");
    message += fl;
    message += F("TimeOut Wifi ");
    message += config.timeoutWifi;
    message += " s";
  }
  else {
    message += F("Connexion Wifi impossible");
  }
  sendReply(Origine);

  // A Finir
  // if (sms) { // suppression du SMS
  //   /* Obligatoire ici si non bouclage au redemarrage apres timeoutwifi
  //     ou OTA sms demande Wifi toujours present */
  //   EffaceSMS(index);
  // }
  debut = millis();
  if (!error) {
    /* boucle permettant de faire une mise à jour OTA et serveur, avec un timeout en cas de blocage */
    unsigned long timeout = millis();
    while (millis() - timeout < config.timeoutWifi * 1000) {
      // if(WiFi.status() != WL_CONNECTED) break; // wifi a été coupé on sort
      ArduinoOTA.handle();
      server_wifi.handleClient(); // Listen for client connections
      Alarm.delay(1);
    }
    WifiOff();
  }
}
//---------------------------------------------------------------------------
void ConnexionServeur(String number, bool sms, int index) {
  // connexion Serveur pour UPDATE si disponible
  // lecture fichier .crc
  // lecture fichier .bin
  // si crc calculé en reception = crc lu fichier Update OK
  // effacement du fichier update

  // A Finir
  // uint32_t calculateCRC32 = 0;
  // uint32_t readCRC32 = 0;

  // mqttClient.disconnect();
  // message += "connexion au serveur" + fl;
  // message += "TimeOut (s) = ";
  // message += config.timeoutWifi;
  // sendReply(Origine);

  // if (sms) { // suppression du SMS
  //   /* Obligatoire ici si non bouclage au redemarrage apres timeoutwifi
  //     ou OTA sms demande Wifi toujours present */
  //   EffaceSMS(index);
  // }
  // String Fichier = "/bin/";
  // Fichier += soft;
  // Fichier += String (ver + 1); // cherche version actuelle + 1
  // Serial.print("version cherche:"), Serial.println(Fichier);
  // uint32_t contentLength = 0;
  // uint32_t readLength = 0;
  // unsigned long timeElapsed = millis();
  // String receivecrc = "";

  // // ouverture fichier .crc
  // if (client.connect(config.mqttServer, 80)) {
  //   Serial.println("connexion server ok");
  //   unsigned long timeout = millis();
  //   client.print(String("GET ") + Fichier + ".crc" + " HTTP/1.0\r\n");
  //   client.print(String("Host: ") + config.mqttServer + "\r\n");
  //   client.print("Connection: close\r\n\r\n");
  //   while (client.available() == 0) {
  //     if (millis() - timeout > 5000L) {
  //       Serial.println(">>> Client Timeout !");
  //       client.stop();
  //       Alarm.delay(10000L);
  //       return;
  //     }
  //   }
  //   Serial.println("receiving Header");
  //   while (client.available()) {
  //     String line = client.readStringUntil('\n');
  //     line.trim();
  //     // Serial.println(line);    // Uncomment this to show response header
  //     line.toLowerCase();
  //     if (line.startsWith("content-length:"))
  //     {
  //       contentLength = line.substring(line.lastIndexOf(':') + 1).toInt();
  //     }
  //     else if (line.length() == 0)
  //     {
  //       break;
  //     }
  //   }
  //   Serial.println("receiving answer");
  //   timeout = millis();
  //   while (readLength < contentLength && client.connected() && millis() - timeout < 10000L) {
  //     int i = 0;
  //     while (client.available()) {
  //       char c = client.read();
  //       receivecrc += c;
  //       // Serial.print((char)c);       // Uncomment this to show data
  //       readLength++;
  //       timeout = millis();
  //     }
  //   }
  //   char bidon[10];
  //   receivecrc.toCharArray(bidon, receivecrc.length() + 1);
  //   readCRC32 = (int)strtol(bidon, NULL, 16);
  // }
  // else {
  //   Serial.println("connexion server fail");
  //   return;
  // }


  // // ouverture fichier .bin
  // if (client.connect(config.mqttServer, 80)) {
  //   Serial.println("connexion server ok");
  //   unsigned long timeout = millis();

  //   client.print(String("GET ") + Fichier + ".bin" + " HTTP/1.0\r\n");
  //   client.print(String("Host: ") + config.mqttServer + "\r\n");
  //   client.print("Connection: close\r\n\r\n");
  //   while (client.available() == 0) {
  //     if (millis() - timeout > 5000L) {
  //       Serial.println(">>> Client Timeout !");
  //       client.stop();
  //       Alarm.delay(10000L);
  //       return;
  //     }
  //   }

  //   Serial.println("receiving Header");

  //   File file = SPIFFS.open("/update.bin", FILE_WRITE);

  //   while (client.available()) {
  //     String line = client.readStringUntil('\n');
  //     line.trim();
  //     //Serial.println(line);    // Uncomment this to show response header
  //     line.toLowerCase();
  //     if (line.startsWith("content-length:")) {
  //       contentLength = line.substring(line.lastIndexOf(':') + 1).toInt();
  //     }
  //     else if (line.length() == 0) {
  //       break;
  //     }
  //   }

  //   Serial.println("receiving answer");
  //   timeout = millis();
  //   CRC32 crc;
  //   printPercent(readLength, contentLength);

  //   while (readLength < contentLength && client.connected() && millis() - timeout < 10000L) {
  //     while (client.available()) {
  //       char c = client.read();
  //       file.print(c);
  //       // Serial.print((char)c);       // Uncomment this to show data
  //       crc.update(c);
  //       readLength++;
  //       if (readLength % (contentLength / 13) == 0) {
  //         printPercent(readLength, contentLength);
  //       }
  //       timeout = millis();
  //     }
  //   }
  //   file.close();
  //   calculateCRC32 = crc.finalize();

  //   printPercent(readLength, contentLength);
  //   timeElapsed = millis() - timeElapsed;
  //   Serial.println();

  //   client.stop();
  //   Serial.println("Disconnected from Server");

  //   float duration = float(timeElapsed) / 1000;

  //   Serial.print("File Size: "), Serial.println(contentLength);
  //   Serial.print("Read:  "), Serial.println(readLength);
  //   Serial.print("Read       CRC32: 0x"), Serial.println(readCRC32, HEX);
  //   Serial.print("Calculated CRC32: 0x"), Serial.println(calculateCRC32, HEX);
  //   Serial.print("Download in:      "), Serial.print(duration), Serial.println("s");

  //   if (calculateCRC32 == readCRC32) {
  //     Serial.println("crc ok:");
  //     file = SPIFFS.open("/update.bin");
  //     if (!file) {
  //       Serial.println("Failed to open file for reading");
  //       return;
  //     }
  //     Serial.println("Starting update..");
  //     size_t fileSize = file.size();
  //     if (!Update.begin(fileSize)) {
  //       Serial.println("Cannot do the update");
  //       return;
  //     }
  //     Update.writeStream(file);
  //     if (Update.end()) {
  //       Serial.println("Successful update");
  //     } else {
  //       Serial.println("Error Occurred: " + String(Update.getError()));
  //       return;
  //     }
  //     file.close();
  //     SPIFFS.remove("/update.bin");
  //     Serial.println("Reset in 4 seconds...");
  //     Alarm.delay(4000);
  //     ESP.restart();
  //   }
  //   else {
  //     Serial.println("erreur chargement crc KO:");
  //   }
  // }
  // else {
  //   Serial.println("connexion server fail");
  //   return;
  // }
  // Serial.println("Lancement reset");
  // FlagReset = true;
}
//---------------------------------------------------------------------------
void WifiOff() {
  Serial.println(F("Wifi off"));
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_MODE_NULL);
  btStop();
  Alarm.delay(100);
  ResetHard();
}
//---------------------------------------------------------------------------
void ResetHard() {
  delay(100);// imperatif
  ESP.restart();

  // GPIO13 to RS reset hard
  pinMode(PinReset, OUTPUT);
  digitalWrite(PinReset, LOW);
}
//---------------------------------------------------------------------------
void OuvrirPB() { // Vérification fichier PhoneBook existe
  // par ligne : N°tel;Nom\n
  if (!SPIFFS.exists(filePhoneBook)) {
    // fichier n'existe pas
    Serial.print(F("Creating Data File:")), Serial.println(filePhoneBook); // valeur par defaut
    File f = SPIFFS.open(filePhoneBook, "w+");
    f.println(default_PB);
    f.close();
  }
}
//---------------------------------------------------------------------------
void OuvrirFichierCalibration() { // Lecture fichier calibration

  if (SPIFFS.exists(filecalibration)) {
    File f = SPIFFS.open(filecalibration, "r");
    for (int i = 0; i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])); i++) { //Read
      String s = f.readStringUntil('\n');
      CoeffTension[i] = s.toFloat();
    }
    f.close();
  }
  else {
    Serial.print(F("Creating Data File:")), Serial.println(filecalibration); // valeur par defaut
    CoeffTension[0] = CoeffTensionDefaut;
    CoeffTension[1] = CoeffTensionDefaut;
    // CoeffTension[2] = CoeffTensionDefaut;
    // CoeffTension[3] = CoeffTensionDefaut;
    Recordcalib();
  }
  Serial.print(F("Coeff T Batterie = ")), Serial.print(CoeffTension[0]);
  Serial.print(F(" Coeff T Proc = "))	  , Serial.print(CoeffTension[1]);
  // Serial.print(F(" Coeff T VUSB = "))		, Serial.print(CoeffTension[2]);
  // Serial.print(F(" Coeff T 24V = "))		, Serial.println(CoeffTension[3]);

}
//---------------------------------------------------------------------------
void RecordPB(String newline) { // enregistrer fichier Phone Book en SPIFFS
  
}
//---------------------------------------------------------------------------
void Recordcalib() { // enregistrer fichier calibration en SPIFFS
  // Serial.print(F("Coeff T Batterie = ")),Serial.println(CoeffTension1);
  // Serial.print(F("Coeff T Proc = "))	  ,Serial.println(CoeffTension2);
  // Serial.print(F("Coeff T VUSB = "))		,Serial.println(CoeffTension3);
  File f = SPIFFS.open(filecalibration, "w");
  f.println(CoeffTension[0]);
  f.println(CoeffTension[1]);
  // f.println(CoeffTension[2]);
  // f.println(CoeffTension[3]);
  f.close();
}
//--------------------------------------------------------------------------------//
void messageId() {
  message = Id;
  message += displayTime();
  message += fl;
}
//---------------------------------------------------------------------------
void HomePage() {
  SendHTML_Header();
  webpage += F("<h3 class='rcorners_m'>Parametres</h3><br>");
  webpage += F("<table align='center'>");
  webpage += F("<tr>");
  webpage += F("<td>Version</td>");
  webpage += F("<td>");	webpage += ver;	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Id</td>");
  webpage += F("<td>");	webpage += String(config.Idchar);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Tempo Send Data GPS Rapide (5-3600s)</td>");
  webpage += F("<td>");	webpage += String(config.trapide);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Tempo Send Data GPS Lent (10-3600s)</td>");
  webpage += F("<td>");	webpage += String(config.tlent);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Vitesse mini (1-20kmh)</td>");
  webpage += F("<td>");	webpage += String(config.vtransition);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>APN GPRS</td>");
  webpage += F("<td>");	webpage += String(config.apn);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>GPRS User</td>");
  webpage += F("<td>");	webpage += String(config.gprsUser);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>GPRS Pass</td>");
  webpage += F("<td>");	webpage += String(config.gprsPass);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>MQTT Serveur</td>");
  webpage += F("<td>");	webpage += String(config.mqttServer);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>MQTT User</td>");
  webpage += F("<td>");	webpage += String(config.mqttUserName);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>MQTT Pass</td>");
  webpage += F("<td>");	webpage += String(config.mqttPass);	webpage += F("</td>");
  webpage += F("</tr>");

  // webpage += F("<tr>");
  // webpage += F("<td>MQTT Topic</td>");
  // webpage += F("<td>");	webpage += String(config.writeTopic);	webpage += F("</td>");
  // webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>TimeOut Wifi (s)</td>");
  webpage += F("<td>");	webpage += String(config.timeoutWifi);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("</table><br>");

  webpage += F("<a href='/download'><button>Download</button></a>");
  webpage += F("<a href='/upload'><button>Upload</button></a>");
  webpage += F("<a href='/delete'><button>Delete</button></a>");
  webpage += F("<a href='/dir'><button>Directory</button></a>");
  webpage += F("<a href='/Tel_list'><button>Tel_list</button></a>");
  // webpage += F("<a href='/cal'><button>Calendar</button></a>");
  // webpage += F("<a href='/LumLUT'><button>LumLUT</button></a>");
  webpage += F("<a href='/wifioff'><button>Wifi Off</button></a>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
void Tel_listPage() {
  SendHTML_Header();
  webpage += F("<h3 class='rcorners_m'>Liste des num&eacute;ros t&eacute;l&eacute;phone</h3><br>");
  webpage += F("<table align='center'>");
  webpage += F("<tr>");
  webpage += F("<th> Nom </th>");
  webpage += F("<th> Num&eacute;ro </th>");
  webpage += F("</tr>");

  File file = SPIFFS.open(filePhoneBook, "r");
  while (file.available()) {
    String ligne = file.readStringUntil('\n');
    byte pos1 = ligne.indexOf(";");
    String number   = ligne.substring(0,pos1);
    String name = ligne.substring(pos1+1,ligne.length()-1);
    webpage += F("<tr>");
    webpage += F("<td>"); webpage += String(name); webpage += F("</td>");
    webpage += F("<td>"); webpage += String(number); webpage += F("</td>");
    webpage += F("</tr>");
  }
  file.close();
webpage += F("</table><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Download() { // This gets called twice, the first pass selects the input, the second pass then processes the command line arguments
  if (server_wifi.args() > 0 ) { // Arguments were received
    if (server_wifi.hasArg("download")) DownloadFile(server_wifi.arg(0));
  }
  else SelectInput("Enter filename to download", "download", "download");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void DownloadFile(String filename) {
  if (SPIFFS_present) {
    File download = SPIFFS.open("/" + filename,  "r");
    if (download) {
      server_wifi.sendHeader("Content-Type", "text/text");
      server_wifi.sendHeader("Content-Disposition", "attachment; filename=" + filename);
      server_wifi.sendHeader("Connection", "close");
      server_wifi.streamFile(download, "application/octet-stream");
      download.close();
    } else ReportFileNotPresent("download");
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Upload() {
  append_page_header();
  webpage += F("<h3>Select File to Upload</h3>");
  webpage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
  webpage += F("<input class='buttons' style='width:40%' type='file' name='fupload' id = 'fupload' value=''><br>");
  webpage += F("<br><button class='buttons' style='width:10%' type='submit'>Upload File</button><br>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  server_wifi.send(200, "text/html", webpage);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void handleFileUpload() { // upload a new file to the Filing system
  HTTPUpload& uploadfile = server_wifi.upload(); // See https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer/srcv
  // For further information on 'status' structure, there are other reasons such as a failed transfer that could be used
  if (uploadfile.status == UPLOAD_FILE_START)
  {
    String filename = uploadfile.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.print(F("Upload File Name: ")); Serial.println(filename);
    SPIFFS.remove(filename);                  // Remove a previous version, otherwise data is appended the file again
    UploadFile = SPIFFS.open(filename, "w");  // Open the file for writing in SPIFFS (create it, if doesn't exist)
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
    if (UploadFile) UploadFile.write(uploadfile.buf, uploadfile.currentSize); // Write the received bytes to the file
  }
  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if (UploadFile)         // If the file was successfully created
    {
      UploadFile.close();   // Close the file again
      Serial.print(F("Upload Size: ")); Serial.println(uploadfile.totalSize);
      webpage = "";
      append_page_header();
      webpage += F("<h3>File was successfully uploaded</h3>");
      webpage += F("<h2>Uploaded File Name: "); webpage += uploadfile.filename + "</h2>";
      webpage += F("<h2>File Size: "); webpage += file_size(uploadfile.totalSize) + "</h2><br>";
      append_page_footer();
      server_wifi.send(200, "text/html", webpage);
      // OuvrirCalendrier();
    }
    else
    {
      ReportCouldNotCreateFile("upload");
    }
  }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_dir() {
  if (SPIFFS_present) {
    File root = SPIFFS.open("/");
    if (root) {
      root.rewindDirectory();
      SendHTML_Header();
      webpage += F("<h3 class='rcorners_m'>SPIFFS Contents</h3><br>");
      webpage += F("<table align='center'>");
      webpage += F("<tr><th>Name/Type</th><th style='width:20%'>Type File/Dir</th><th>File Size</th></tr>");
      printDirectory("/", 0);
      webpage += F("</table>");
      SendHTML_Content();
      root.close();
    }
    else
    {
      SendHTML_Header();
      webpage += F("<h3>No Files Found</h3>");
    }
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();   // Stop is needed because no content length was sent
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void printDirectory(const char * dirname, uint8_t levels) {
  File root = SPIFFS.open(dirname);
  if (!root) {
    return;
  }
  if (!root.isDirectory()) {
    return;
  }
  File file = root.openNextFile();
  while (file) {
    if (webpage.length() > 1000) {
      SendHTML_Content();
    }
    if (file.isDirectory()) {
      webpage += "<tr><td>" + String(file.isDirectory() ? "Dir" : "File") + "</td><td>" + String(file.name()) + "</td><td></td></tr>";
      printDirectory(file.name(), levels - 1);
    }
    else
    {
      webpage += "<tr><td>" + String(file.name()) + "</td>";
      webpage += "<td>" + String(file.isDirectory() ? "Dir" : "File") + "</td>";
      webpage += "<td>" + file_size(file.size()) + "</td></tr>";
    }
    file = root.openNextFile();
  }
  file.close();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Delete() {
  if (server_wifi.args() > 0 ) { // Arguments were received
    if (server_wifi.hasArg("delete")) SPIFFS_file_delete(server_wifi.arg(0));
  }
  else SelectInput("Select a File to Delete", "delete", "delete");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_file_delete(String filename) { // Delete the file
  if (SPIFFS_present) {
    SendHTML_Header();
    File dataFile = SPIFFS.open("/" + filename, "r"); // Now read data from SPIFFS Card
    if (dataFile)
    {
      if (SPIFFS.remove("/" + filename)) {
        Serial.println(F("File deleted successfully"));
        webpage += "<h3>File '" + filename + "' has been erased</h3>";
        webpage += F("<a href='/delete'>[Back]</a><br><br>");
      }
      else
      {
        webpage += F("<h3>File was not deleted - error</h3>");
        webpage += F("<a href='delete'>[Back]</a><br><br>");
      }
    } else ReportFileNotPresent("delete");
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Header() {
  server_wifi.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server_wifi.sendHeader("Pragma", "no-cache");
  server_wifi.sendHeader("Expires", "-1");
  server_wifi.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server_wifi.send(200, "text/html", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
  append_page_header();
  server_wifi.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Content() {
  server_wifi.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Stop() {
  server_wifi.sendContent("");
  server_wifi.client().stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SelectInput(String heading1, String command, String arg_calling_name) {
  SendHTML_Header();
  webpage += F("<h3>"); webpage += heading1 + "</h3>";
  webpage += F("<FORM action='/"); webpage += command + "' method='post'>"; // Must match the calling argument e.g. '/chart' calls '/chart' after selection but with arguments!
  webpage += F("<input type='text' name='"); webpage += arg_calling_name; webpage += F("' value=''><br>");
  webpage += F("<type='submit' name='"); webpage += arg_calling_name; webpage += F("' value=''><br><br>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportSPIFFSNotPresent() {
  SendHTML_Header();
  webpage += F("<h3>No SPIFFS Card present</h3>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportFileNotPresent(String target) {
  SendHTML_Header();
  webpage += F("<h3>File does not exist</h3>");
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportCouldNotCreateFile(String target) {
  SendHTML_Header();
  webpage += F("<h3>Could Not Create Uploaded File (write-protected?)</h3>");
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
String file_size(int bytes) {
  String fsize = "";
  if (bytes < 1024)                      fsize = String(bytes) + " B";
  else if (bytes < (1024 * 1024))        fsize = String(bytes / 1024.0, 3) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
  else                                   fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";
  return fsize;
}
//---------------------------------------------------------------------------
void handleTime() { // getion temps restant page web
  char time_str[9];
  const uint32_t millis_in_day    = 1000 * 60 * 60 * 24;
  const uint32_t millis_in_hour   = 1000 * 60 * 60;
  const uint32_t millis_in_minute = 1000 * 60;

  static unsigned long t0 = 0;
  if (millis() - debut > config.timeoutWifi * 1000) debut = millis(); // securité evite t<0
  t0 = debut + (config.timeoutWifi * 1000) - millis();
  // Serial.print(debut),Serial.print("|"),Serial.println(t0);

  uint8_t days     = t0 / (millis_in_day);
  uint8_t hours    = (t0 - (days * millis_in_day)) / millis_in_hour;
  uint8_t minutes  = (t0 - (days * millis_in_day) - (hours * millis_in_hour)) / millis_in_minute;
  uint8_t secondes = (t0 - (days * millis_in_day) - ((hours * millis_in_hour)) / millis_in_minute) / 1000 % 60;
  sprintf(time_str, "%02d:%02d:%02d", hours, minutes, secondes);
  // Serial.println(time_str);
  server_wifi.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}
//---------------------------------------------------------------------------
void handleDateTime() { // getion Date et heure page web
  char time_str[20];
  sprintf(time_str, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  server_wifi.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}
//---------------------------------------------------------------------------
void PrintConfig() {
  Serial.print(F("Version = "))                 , Serial.println(ver);
  Serial.print(F("ID = "))                      , Serial.println(config.Idchar);
  Serial.print(F("magic = "))                   , Serial.println(config.magic);
  Serial.print(F("apn = "))                     , Serial.println(config.apn);
  Serial.print(F("gprsUser = "))                , Serial.println(config.gprsUser);
  Serial.print(F("gprspass = "))                , Serial.println(config.gprsPass);
  Serial.print(F("mqttServeur = "))             , Serial.println(config.mqttServer);
  Serial.print(F("mqttPort = "))                , Serial.println(config.mqttPort);
  Serial.print(F("mqttUserName = "))            , Serial.println(config.mqttUserName);
  Serial.print(F("mqttPass = "))                , Serial.println(config.mqttPass);
  Serial.print(F("Input Topic 0 = "))           , Serial.println(config.sendTopic[0]);
  Serial.print(F("Input Topic 1 = "))           , Serial.println(config.sendTopic[1]);
  Serial.print(F("Output Topic 0 = "))          , Serial.println(config.recvTopic[0]);
  Serial.print(F("Output Topic 1 = "))          , Serial.println(config.recvTopic[1]);
  Serial.print(F("Tempo rapide = "))            , Serial.println(config.trapide);
  Serial.print(F("Tempo lente = "))             , Serial.println(config.tlent);
  Serial.print(F("Vitesse mini = "))            , Serial.println(config.vtransition);
  Serial.print(F("Time Out Wifi (s)= "))        , Serial.println(config.timeoutWifi);
}
//---------------------------------------------------------------------------
float calc_dist(float Lat1, float Long1, float Lat2, float Long2) {
  // Calcul distance entre 2 point GPS Lat1, Long1, Lat2, Long2
  //  Serial.print(Lat1, 7), Serial.print(F("|")), Serial.println(Long1, 7);
  //  Serial.print(Lat2, 7), Serial.print(F("|")), Serial.println(Long2, 7);
  float dist_calc = 0;
  float dist_calc2 = 0;
  float diflat = 0;
  float diflon = 0;

  diflat = radians(Lat2 - Lat1);
  Lat1 = radians(Lat1);
  Lat2 = radians(Lat2);
  diflon = radians((Long2) - (Long1));

  dist_calc = (sin(diflat / 2.0) * sin(diflat / 2.0));
  dist_calc2 = cos(Lat1);
  dist_calc2 *= cos(Lat2);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc += dist_calc2;

  dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));
  dist_calc *= 6378137.0; //Convertion en metres IAG-GRS80

  //Serial.println(dist_calc);
  return dist_calc;
}
//---------------------------------------------------------------------------
// Envoyer une réponse
// Origine = Local, BLE, SMS, MQTTS (serveur), MQTTU (user)
void sendReply(String Origine) {
  
  if (Origine == "MQTTS"){ // reponse MQTT      
    Envoyer_MQTT(true); // Serveur
  }
  else if (Origine == "MQTTU"){ // reponse MQTT
    Envoyer_MQTT(false); // User
  }
  else if (Origine == "SMS"){
    if (config.sendSMS){
      // A finir
    }
  }

  opmessage = message;
  Serial.println(F("****************************"));
  logmessage(); // Serial.println(message);
  Serial.println(F("****************************"));
}
//---------------------------------------------------------------------------
// Envoyer message en MQTT
// destinataire = true Serveur, false à User
void Envoyer_MQTT(bool dest){
  Serial.println("Sending MQTT len:");
  Serial.println(message.length());
  Serial.print(F("to:"));
  Serial.print(dest ? config.sendTopic[0] : config.sendTopic[1]);
  Serial.print(F(":"));
  if(Online){
    if(dest){ // message Serveur
      if (mqttClient.publish(config.sendTopic[0], message.c_str()), true){ // Serveur
        AlarmeMQTT = false;
        Serial.println(F("OK"));
      } else {
        Serial.println(F("KO"));
        AlarmeMQTT = true;
      }
    } else { // message user
      if(mqttClient.publish(config.sendTopic[1], message.c_str()), true){ // User
        AlarmeMQTT = false;
        Serial.println(F("OK"));
      } else {
        Serial.println(F("KO"));
        AlarmeMQTT = true;
      }
    }
  } else {Serial.println("Off Line");}
}
//---------------------------------------------------------------------------
// force = true, force mise à l'heure systeme sur heure modem, meme si defaut NTP
void MajHeure(bool force) {
  static bool First = true;
  SyncHeureModem(config.hete*4, true);
  if (First) {															  // premiere fois apres le lancement
    readmodemtime();	// lire l'heure du modem
    setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino
    if(!HeureEte()){
      // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hhiver*4));
      SyncHeureModem(config.hhiver*4, true);
      readmodemtime();	// lire l'heure du modem
      setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino
    }
    First = false;
  } else {
    //  calcul décalage entre H sys et H reseau en s
    // resynchroniser H modem avec reseau
    if(HeureEte()){
      // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hete*4));
      if(!SyncHeureModem(config.hete*4, false)){
        if(!force)return; // sortie sans mise à l'heure, continue si forcé
      }
    } else {
      // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hhiver*4));
      if(!SyncHeureModem(config.hhiver*4, false)){
        if(!force)return; // sortie sans mise à l'heure, continue si forcé
      }
    }
    readmodemtime();	// lire l'heure du modem
    int ecart = (N_H - hour()) * 3600;
    ecart += (N_m - minute()) * 60;
    ecart += N_S - second();
    Serial.print(F("Ecart s= ")), Serial.println(ecart);

    if (abs(ecart) > 5) {
      // Arret Alarm en cours
      Alarm.disable(loopPrincipale);
      Alarm.disable(Send);
      setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino
      readmodemtime();	// lire l'heure du modem
      setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino
      // Redemarrage Alarm en cours
      Alarm.write(loopPrincipale, 15);
      if(config.tracker) Alarm.write(Send, config.tlent);
    }
  }
}
//---------------------------------------------------------------------------
// lire l'heure modem
void readmodemtime(){
  String modemHDtate = modem.getGSMDateTime(TinyGSMDateTimeFormat(0));
  // convertir format date time yy/mm/dd,hh:mm:ss
  byte i 	= modemHDtate.indexOf("/");
  byte j 	= modemHDtate.indexOf("/", i + 1);
  N_Y		  = modemHDtate.substring(i - 2, i).toInt();
  N_M 		= modemHDtate.substring(i + 1, j).toInt();
  N_D 		= modemHDtate.substring(j + 1, j + 3).toInt();
  i 	  	= modemHDtate.indexOf(":", 6);
  j     	= modemHDtate.indexOf(":", i + 1);
  N_H 		= modemHDtate.substring(i - 2, i).toInt();
  N_m 		= modemHDtate.substring(i + 1, j).toInt();
  N_S 		= modemHDtate.substring(j + 1, j + 3).toInt();
}
//---------------------------------------------------------------------------
// Set systeme time to modem time
void set_system_time(){
  readmodemtime();
  if(N_Y > 20 && N_Y!= 80){
    // année > 2020 et n'est pas égale à 2080(date defaut modem si pas à l'heure)
    setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure du systeme
    Serial.println("MajHeure du systeme OK");
    FlagSetTime = true;
  } else {
    // echec Synchro, force date 01/08/2022 08:00:00, jour toujours circulé
    setTime(8,0, 0, 1, 8, 22);	    // mise à l'heure du systeme
    // modem.send_AT("+CCLK=\"22/08/01,08:00:00+08\"");
    Serial.println("MajHeure du systeme KO");
    FlagSetTime = false;
  }
}
//---------------------------------------------------------------------------
// start true -> Start Alarmes
// start false-> Stop Alarmes
void StartStopAlarme(bool start){
  if (start){
    Alarm.enable(loopPrincipale);
  } else {
    Alarm.disable(loopPrincipale);
  }
}
//---------------------------------------------------------------------------
// return true en été, false en hiver (1=dimanche)
bool HeureEte() {
  bool Hete = false;
  if (month() > 10 || month() < 3
      || (month() == 10 && (day() - weekday()) > 22)
      || (month() == 3  && (day() - weekday()) < 24)) {
    Hete = false;                      								// c'est l'hiver
  }
  else {
    Hete = true;                       								// c'est l'été
  }
  return Hete;
}
//---------------------------------------------------------------------------
String displayTime() {
  char bidon[20];
  sprintf(bidon, "%02d/%02d/%04d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  return String(bidon);
}
//---------------------------------------------------------------------------
// lire valeur RSSI et remplir message
String read_RSSI() {

  String rssi = "";
  int r;
  byte n = modem.getSignalQuality();
  // Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  rssi  = F("RSSI= ");
  rssi += String(n);
  rssi += ", ";
  rssi += String(r);
  rssi += F("dBm");
  return rssi;
}
//---------------------------------------------------------------------------
void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory())
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available())
  {
    Serial.write(file.read());
    delayMicroseconds(100);
  }
}
//---------------------------------------------------------------------------
// etat synchronisation time/heure systeme
void timesstatus() {
  Serial.print(F("Synchro Time  : "));
  switch (timeStatus()) {
    case 0:
      Serial.println(F(" pas synchro"));
      break;
    case 1:
      Serial.println(F(" defaut synchro"));
      break;
    case 2:
      Serial.println(F(" OK"));
      break;
  }
}
//---------------------------------------------------------------------------
// mise forme date/time
String displayTime(byte n) {
  // n = 0 ; dd/mm/yyyy hh:mm:ss
  // n = 1 ; yyyy-mm-dd hh:mm:ss
  char bid[20];
  if (n == 0) {
    sprintf(bid, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  }
  else {
    sprintf(bid, "%4d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
  }
  return String(bid);
}
//---------------------------------------------------------------------------
bool ConnectGPRS(){
  opmessage += "Connecting to ";
  Serial.print(config.apn);
  if (modem.gprsConnect(config.apn, config.gprsUser, config.gprsPass)) {
    opmessage += ". success";
    return true;
  }
  else {
    opmessage += " fail";
    return false;
  }
  // logmessage();
}
//---------------------------------------------------------------------------
void printPercent(uint32_t readLength, uint32_t contentLength) {
  // If we know the total length
  if (contentLength != -1)
  {
    Serial.print("\r ");
    Serial.print((100.0 * readLength) / contentLength);
    Serial.print('%');
  }
  else
  {
    Serial.println(readLength);
  }
}
//---------------------------------------------------------------------------
int modem_on() {
    /*
    The indicator light of the board can be controlled
    */
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    /*
    MODEM_PWRKEY IO:4 The power-on signal of the modulator must be given to it,
    otherwise the modulator will not reply when the command is sent
    */
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, HIGH);// LOW sur pin Power key
    delay(300); //Need delay
    digitalWrite(MODEM_PWRKEY, LOW); // HIGH sur pin Power key

    /*
    MODEM_FLIGHT IO:25 Modulator flight mode control,
    need to enable modulator, this pin must be set to high
    */
    pinMode(MODEM_FLIGHT, OUTPUT);
    digitalWrite(MODEM_FLIGHT, HIGH);

  int i = 10;
  Serial.println("\nTesting Modem Response...\n");
  Serial.println("****");
  while (i) {
    SerialAT.println("AT");
    delay(500);
    if (SerialAT.available()) {
      String r = SerialAT.readString();
      Serial.println(r);
      if ( r.indexOf("OK") >= 0 ) {
        // reply = true;
        break;
      }
    }
    delay(500);
    i--;
  }
  Serial.println("****");
  return i;
}
  //---------------------------------------------------------------------------
// Cherche number existe dans fichier PhoneBook
bool Cherche_N_PB(String number){
  if (SPIFFS.exists(filePhoneBook)) {
    File file = SPIFFS.open(filePhoneBook, "r");
    while (file.available()) {
      String s = file.readStringUntil('\n');
      if(s.indexOf(number)>-1){
        Serial.print("N° trouve:"),Serial.println(s);
        file.close();
        return true;
      }
    }
    file.close();
  }
  return false;
}
//---------------------------------------------------------------------------
bool SyncHeureModem(int Savetime, bool FirstTime){
  int rep = 1;
  int compteur = 0;
  if(modem.isNetworkConnected() && modem.isGprsConnected()){
    do{
      rep = modem.NTPServerSync(NTPServer, Savetime);
      Serial.print("NTP:"),Serial.println(modem.ShowNTPError(rep));
      if (compteur > 10){ // 10 tentatives
        if(FirstTime){
          // echec Synchro, force date 01/08/2022 08:00:00, jour toujours circulé
          modem.send_AT("+CCLK=\"22/08/01,08:00:00+08\"");
          delay(500);
        }
        return false;
      }
      compteur += 1;
      delay(1000);
    } while(rep != 1);
    return true;
  }
  return false;
}
//---------------------------------------------------------------------------
// void SignalVie() {
//   Serial.print(F("SignalVie "));
//   displayTime();
//   envoieGroupeMessage(true);		// envoie groupé
//   NbrResetModem = 0;          // reset compteur reset modem
// }
//---------------------------------------------------------------------------
// retourne n° derniere ligne PB
byte last_PB(){
  Read_PB();
  byte dernier = 0;
  for(byte i = 1;i<10;i++){
    if(strlen(PB_list[i]) > 0){
      dernier = i;
    }
  }
  return dernier;
}
//---------------------------------------------------------------------------
// Vérification fichier PhoneBook existe
void Ouvrir_PB() {
  // par ligne : N°tel;Nom\n
  if (!SPIFFS.exists(filePhoneBook)) {
    // fichier n'existe pas
    Serial.print(F("Creating Data File:")), Serial.println(filePhoneBook); // valeur par defaut
    File file = SPIFFS.open(filePhoneBook, "w+");
    file.println(default_PB);
    file.close();
  }
  Read_PB();
}
//---------------------------------------------------------------------------
// Lecture filePhoneBook copie dans PB_list
void Read_PB(){
  // vide PB_list
  for(byte i = 1;i<10;i++){
    strcpy(PB_list[i] , "");
  }
  // lire fichier
  File file = SPIFFS.open(filePhoneBook, "r");
  byte idx = 0;
  while (file.available()) {
    idx ++;
    String ligne = file.readStringUntil('\n');
    strcpy(PB_list[idx] , ligne.c_str());
  }
  file.close();
}
//---------------------------------------------------------------------------
// Sauvegarde filePhoneBook
void Save_PB(){
  File file = SPIFFS.open(filePhoneBook, "w+");
  for (byte i = 1;i<10;i++){
    if(strlen(PB_list[i])>0){
      file.println(String(PB_list[i]));
    }
  }
  file.close();
}
//---------------------------------------------------------------------------
void createSDFile(){
  // creation fichier date du jour
  char datefile[14];
  sprintf(datefile, "/%02d%02d%02d%02d.txt", month(),	day(), hour(), minute());      
  SDfilename = String(datefile);								//jjhhmm.txt
  Serial.print("creation fichier SD:"),Serial.println(SDfilename);
  File file = SD.open(SDfilename, "w+");				//creation fichier
  file.close();
}
//---------------------------------------------------------------------------
// Read Config
void readConfig(){
  Serial.printf("Reading file: %s\r\n", fileconfig);

  File file = SPIFFS.open(fileconfig);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return;
  }

  Serial.println("- read from file:");
  file.read((byte *)&config, sizeof(config));
  file.close();
}
//---------------------------------------------------------------------------
// Sauvegarde Config
void sauvConfig(){
  Serial.printf("Writing file: %s\r\n", fileconfig);

  File file = SPIFFS.open(fileconfig, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.write((byte *)&config, sizeof(config))){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}
//---------------------------------------------------------------------------
void logmessage(){
  static bool first = true;
  if(FlagSDCardPresent && config.logSDcard){
    if(first){
      first = false;
      createSDFile();
    }
    // ecrire sur carte SD
    File file = SD.open(SDfilename,FILE_APPEND);
    file.print(displayTime());
    file.print(";");
    file.println(opmessage);
    file.close();
  }
  Serial.println(opmessage);
}
//---------------------------------------------------------------------------
// Copie valeur topic en config
void copie_Topic(){
  strncpy(config.sendTopic[0],("L/in"),sizeof(config.sendTopic[0]));   // localisation vers Serveur
  strncpy(config.sendTopic[1],("L/Uin/"),sizeof(config.sendTopic[1])); // vers User
  strcat( config.sendTopic[1], &config.Idchar[5]); // L/Uin/TTX0x ou L/Uin/VR00x

  strncpy(config.recvTopic[0],("L/Sout/"),sizeof(config.recvTopic[0])); // from Serveur
  strcat( config.recvTopic[0], &config.Idchar[5]); // L/Sout/TTX0x ou VR00x
  
  strncpy(config.recvTopic[1],("L/Uout/"),sizeof(config.recvTopic[1])); // from User
  strcat( config.recvTopic[1], &config.Idchar[5]); // L/Sout/TTX0x ou VR00x
}
//---------------------------------------------------------------------------
// Change Blinker
void Change(bool on){
  Blinker.detach();
  if(!on){
    BlinkerInterval = 1000;
    Blinker.attach_ms(BlinkerInterval, Blink);
  } else {
    BlinkerInterval = 200;
    Blinker.attach_ms(BlinkerInterval, Blink);
  }
  Serial.print("change Blinker:"),Serial.println(BlinkerInterval);
}
//---------------------------------------------------------------------------
// Blinker
void Blink(){
  digitalWrite(LED_EXTERNE_PIN, !digitalRead(LED_EXTERNE_PIN));
}
/* --------------------  test local serial seulement ----------------------*/
void recvOneChar() {

  char   receivedChar;
  static String serialmessage = "";
  static bool   newData = false;

  if (Serial.available() > 0) {
    receivedChar = Serial.read();
    if (receivedChar != 10 && receivedChar != 13) {
      serialmessage += receivedChar;
    }
    else {
      newData = true;
      return;
    }
  }
  if (newData == true) {
    Serial.println(serialmessage);
    Rmessage = serialmessage;
    Serial.flush();
    traite_sms("Local");//	traitement en mode local
    newData = false;
    serialmessage = "";
  }
}

