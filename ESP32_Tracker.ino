/* Tracker GPRS MQTT */
/* Ph Corbel 31/01/2020 */
/* ESP32+Sim808
  Compilation LOLIN D32,default,80MHz, ESP32 1.0.2 (1.0.4 bugg?)
  Arduino IDE 1.8.10 : 966722 73%, 46880 14% sur PC
  Arduino IDE 1.8.10 :  76%,  14% sur raspi


*/

/* TinyGsmClient.h
  https://github.com/vkc2019/gps-tracker-sim808-esp32
  gestion SMS et PhoneBook
  https://github.com/szotsaki/TinyGSM/blob/master/src/TinyGsmClientSIM800.h
  sms
  https://github.com/mayurharge/TinyGSM/blob/master/src/TinyGsmClientSIM800.h
*/

#include <ArduinoJson.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <sys/time.h>             //<sys/time.h>
#include <EEPROM.h>               // variable en EEPROM
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <FS.h>
#include <SPI.h>
#include <Ticker.h>
#include "passdata.h"

#define TINY_GSM_MODEM_SIM808
#define Serial Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG Serial
#define TINY_GSM_YIELD() { delay(2); }
#define TINY_GSM_USE_GPRS true
// #define TINY_GSM_USE_WIFI false
#define GSM_PIN "1234"

String  webpage = "";
#define ServerVersion "1.0"
bool    SPIFFS_present = false;
#include "CSS.h"               // pageweb

#define RXD2 16
#define TXD2 17
#define RESET_PIN     18   // Rest Sim 808
#define PinReset      13   // Reset Hard
#define PinBattProc   35   // liaison interne carte Lolin32 adc

const String soft = "ESP32_Tracker.ino.d32"; // nom du soft
String ver        = "V1-0";
int    Magique    = 11;

char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
#define nSample (1<<4)    // nSample est une puissance de 2, ici 16 (4bits)
unsigned int adc_hist[5][nSample]; // tableau stockage mesure adc, 0 Batt, 1 Proc, 2 USB, 3 24V, 5 Lum
unsigned int adc_mm[5];            // stockage pour la moyenne mobile
unsigned long debut     = 0; // pour decompteur temps wifi
unsigned long timer100  = 0; // pour timer 100ms adc
int CoeffTension[4];          // Coeff calibration Tension
int CoeffTensionDefaut = 7000;// Coefficient par defaut
bool FlagReset = false;
byte Accu = 0; // accumulateur vitesse
float lat, lon, course, speed;
long   VBatterieProc    = 0; // Tension Batterie Processeur
String Sbidon 		= ""; // String texte temporaire
String messagetest = "";
String message = "";
String fl = "\n";                   //  saut de ligne SMS
String Id ;                         //  Id du materiel sera lu dans EEPROM
byte confign = 0;                   //  position enregistrement config EEPROM

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

PubSubClient mqttClient(client);

WebServer server_wifi(80);
File UploadFile;

struct  config_t           // Structure configuration sauvée en EEPROM
{
  int     magic;           // num magique
  int     trapide;         // timer send data rapide (roulage)
  int     tlent;           // timer send data lent (arret)
  int     vtransition;     // vitesse transition arret/roulage
  int     timeoutWifi;     // tempo coupure Wifi si pas de mise a jour (s)
  char    apn[11];         // APN
  char    gprsUser[11];    // user for APN
  char    gprsPass[11];    // pass for APN
  char    mqttServer[26];  // Serveur MQTT
  char    mqttUserName[11]; // MQTT User
  char    mqttPass[16];    // MQTT pass
  char    writeTopic[16];  // channel Id
  int     mqttPort;        // Port serveur MQTT
  char    Idchar[11];      // Id
} ;
config_t config;

const int nbrdata = 1;
String fieldsToPublish[nbrdata]; // Change to allow multiple fields.
String dataToPublish[nbrdata];   // Holds your field data.


AlarmId loopPrincipale;    // boucle principale
AlarmId first;             // premier send data
AlarmId send;              // timer send data
Ticker ADC;                // Lecture des Adc

//---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  SerialAT.begin(9600, SERIAL_8N1, RXD2, TXD2, false);
  Alarm.delay(1000);

  init_adc_mm();// initialisation tableau pour adc Moyenne Mobile
  ADC.attach_ms(100, adc_read); // lecture des adc toute les 100ms

  EEPROM.begin(512);
  EEPROM.get(confign, config); // lecture config
  int recordn = sizeof(config);
  Serial.print("len config ="), Serial.println(sizeof(config));
  Alarm.delay(500);
  if (config.magic != Magique) {
    /* verification numero magique si different
    		erreur lecture EEPROM ou carte vierge
    		on charge les valeurs par défaut
    */
    Serial.println(F("Nouvelle Configuration !"));
    config.magic            = Magique;
    config.trapide          = 15;      // secondes
    config.tlent            = 15 * 60; // secondes
    config.vtransition      = 2;       // kmh
    config.timeoutWifi      = 10 * 60;
    String temp             = "TPCF_63000";
    String tempapn          = "sl2sfr";//"free";
    String tempUser         = "";
    String tempPass         = "";
    String tempmqttServer   = "philippeco.hopto.org";
    String tempmqttUserName = "TpcfUser";
    String tempmqttPass     = "hU4zHox1iHCDHM2";
    String temptopic        = "localisation";
    config.mqttPort         = 1883;
    temp.toCharArray(config.Idchar, 11);
    tempapn.toCharArray(config.apn, (tempapn.length() + 1));
    tempUser.toCharArray(config.gprsUser, (tempUser.length() + 1));
    tempPass.toCharArray(config.gprsPass, (tempPass.length() + 1));
    tempmqttServer.toCharArray(config.mqttServer, (tempmqttServer.length() + 1));
    tempmqttUserName.toCharArray(config.mqttUserName, (tempmqttUserName.length() + 1));
    tempmqttPass.toCharArray(config.mqttPass, (tempmqttPass.length() + 1));
    temptopic.toCharArray(config.writeTopic, (temptopic.length() + 1));
    EEPROM.put(confign, config);
    EEPROM.commit();
  }
  EEPROM.end();
  PrintEEPROM();
  Id  = String(config.Idchar);
  Id += fl;

  modem.init(GSM_PIN);

  Serial.println("Initializing modem...");
  String modemInfo = modem.getModemInfo();

  Serial.print("Modem Info: ");
  Serial.println(modemInfo);

  if (modem.enableGPS()) {
    Serial.println("GPS Enabled");
  }

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(" fail");
    Alarm.delay(10000);
    return;
  }
  Serial.println(" success");

  if (modem.isNetworkConnected()) {
    Serial.println("Network connected");
  }

  Serial.print(F("Connecting to "));
  Serial.print(config.apn);
  if (!modem.gprsConnect(config.apn, config.gprsUser, config.gprsPass)) {
    Serial.println(" fail");
    Alarm.delay(10000);
    return;
  }
  Serial.println(F(". success"));
  bool res = modem.isGprsConnected();
  // DBG("GPRS status:", res ? "connected" : "not connected");

  // String ccid = modem.getSimCCID();
  // Serial.print("CCID:"),Serial.println(ccid);

  String imei = modem.getIMEI();
  Serial.println("IMEI:" + imei);

  String cop = modem.getOperator();
  Serial.println("Operator:" + cop);

  IPAddress local = modem.localIP();
  Serial.println("Local IP:" + local.toString());

  Serial.print("Signal quality:"), Serial.println(read_RSSI());

  mqttClient.setServer(config.mqttServer, config.mqttPort); // Set the MQTT broker details.
  //// mqttClient.setCallback( mqttSubscriptionCallback );   // Set the MQTT message handler function.

  while (!MajHeure()) {
    Serial.print(".");
    Alarm.delay(500);
  }
  Serial.println();

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
    delay(100);
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

  if (!SPIFFS.begin(true)) {
    Serial.println(F("SPIFFS initialisation failed..."));
    SPIFFS_present = false;
  }
  else {
    Serial.println(F("SPIFFS initialised... file access enabled..."));
    SPIFFS_present = true;
  }

  OuvrirFichierCalibration(); // ouvre fichier calibration en SPIFFS

  loopPrincipale = Alarm.timerRepeat(10, Acquisition); // boucle principale 10s
  Alarm.enable(loopPrincipale);

  first = Alarm.timerOnce(60, once); // premier lancement
  Alarm.enable(first);

  send = Alarm.timerRepeat(config.tlent, senddata); // send data
  Alarm.disable(send);
}
//---------------------------------------------------------------------------
void loop() {

  recvOneChar();

  if (!mqttClient.connected()) {
    mqttConnect(); // Connect if MQTT client is not connected.
    // if (mqttSubscribe( readChannelID, 1, readAPIKey, 0 ) == 1 ) {
    // Serial.println( " Subscribed " );
    // }
  }

  mqttClient.loop(); // Call the loop to maintain connection to the server.
  Alarm.delay(0);
}
//---------------------------------------------------------------------------
void once() {
  Serial.println("Premier lancement");
  Alarm.enable(send);
  senddata();
}
//---------------------------------------------------------------------------
void senddata() {
  if (getGPSdata()) {
    // mqttPublishThingspeak( writeTopic, writeAPIKey, dataToPublish, fieldsToPublish );
    mqttPublish(config.writeTopic, dataToPublish, fieldsToPublish);
  }
}
//---------------------------------------------------------------------------
void Acquisition() {

  static float lastlon = 0;
  static float lastlat = 0;
  getGPSdata();
  Serial.print("distance:"), Serial.println(calc_dist(lat, lon, lastlat, lastlon), 2);
  lastlat = lat;
  lastlon = lon;
  gereCadence();

  if (CoeffTension[0] == 0 || CoeffTension[1] == 0 || CoeffTension[2] == 0 || CoeffTension[3] == 0) {
    OuvrirFichierCalibration(); // patch relecture des coeff perdu
  }
  VBatterieProc   = map(adc_mm[1] / nSample, 0, 4095, 0, CoeffTension[1]);
  Serial.print("tension proc = "), Serial.print(VBatterieProc / 1000.0, 3), Serial.println("V");
  int idx = modem.newMessageIndex(0); // verifie arrivée sms

  if (idx > 0) {
    Serial.print("sms recu:"), Serial.println(idx);
    traite_sms(idx);
  }
  else if (idx == 0 && FlagReset) {
    FlagReset = false;
    ResetHard();				//	reset hard
  }
}
//---------------------------------------------------------------------------
void traite_sms(int index) {
  bool sms = true;
  String textesms;
  String SenderNum;
  String Sendername;

  /* Variables pour mode calibration */
  static int tensionmemo = 0;//	memorisation tension batterie lors de la calibration
  int coef = 0; // coeff temporaire
  static byte P = 0; // Pin entrée a utiliser pour calibration
  static byte M = 0; // Mode calibration 1,2,3,4
  static bool FlagCalibration = false;	// Calibration Tension en cours

  Serial.print("sms a traiter:"), Serial.println(index);
  if (index == 99) {
    sms = false;
    textesms = messagetest;
  } else {
    Serial.print("newmessageindex: "), Serial.println(index);
    textesms = modem.readSMS(index);    
    SenderNum = modem.getSenderID(index, false);
    Serial.print("sender: "), Serial.println(SenderNum);
    Sendername = modem.getSenderName(index, false);
    // Serial.print("sendername length: "), Serial.println(Sendername.length());
    Serial.print("sendername: "), Serial.println(Sendername);
  }
  Serial.print("sms: "), Serial.println(textesms);
  for (byte i = 0; i < textesms.length(); i++) {
    if ((int)textesms[i] < 0 || (int)textesms[i] > 127) { // caracteres accentués interdit
      goto sortir;
    }
  }
  if ((sms && Sendername.length() > 0) || !sms) { // emetteur reconnu dans Phone Book
    messageId();
    if (!(textesms.indexOf(F("TEL")) == 0 || textesms.indexOf(F("tel")) == 0 || textesms.indexOf(F("Tel")) == 0
          || textesms.indexOf(F("Wifi")) == 0 || textesms.indexOf(F("GPRSDATA")) == 0 || textesms.indexOf(F("MQTTDATA")) == 0)) {
      textesms.toUpperCase();    // passe tout en Maj sauf si "TEL"
      textesms.replace(" ", ""); // supp tous les espaces
    }

    if (textesms.indexOf(F("ID=")) == 0) {			//	Id= nouvel Id
      String temp = textesms.substring(3);
      if (temp.length() > 0 && temp.length() < 11) {
        Id = "";
        temp.toCharArray(config.Idchar, 11);
        sauvConfig();															// sauvegarde en EEPROM
        Id = String(config.Idchar);
        Id += fl;
      }
      message += F("Nouvel Id");
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("TEL")) == 0
             || textesms.indexOf(F("Tel")) == 0
             || textesms.indexOf(F("tel")) == 0) { // entrer nouveau num
      bool FlagOK = true;
      bool efface = false;
      byte j = 0;
      String newnumero;
      String newnom;
      int indexreplace = 0;

      if (textesms.indexOf(char(61)) == 4) { // TELn= reserver correction/suppression
        int i = textesms.substring(3).toInt();// recupere n° de index
        i = i / 1; // important sinon i ne prend pas sa valeur dans les comparaison?
        if (i < 1) FlagOK = false;
        indexreplace = i;// index du PB a remplacer
        j = 5;
        // on efface la ligne sauf la 1 pour toujours garder au moins un numéro
        if ((i != 1) && (textesms.indexOf(F("efface")) == 5 || textesms.indexOf(F("EFFACE")) == 5 )) {
          efface = true;
          bool ok = modem.deletePhonebookEntry(i);
          if (ok) {
            message += "entree PB effacee";
          } else {
            message += "erreur efface entree PB";
          }
          goto fin_tel;
        }
      }
      else if (textesms.indexOf(char(61)) == 3) { // TEL= nouveau numero
        j = 4;
      }
      else {
        FlagOK = false;
      }
      if (textesms.indexOf("+") == j) {			// debut du num tel +
        if (textesms.indexOf(char(44)) == j + 12) {	// verif si longuer ok
          newnumero = textesms.substring(j, j + 12);
          newnom = textesms.substring(j + 13, j + 27);// pas de verif si long<>0?
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
        message += F("Commande non reconnue ?");// non reconnu
        message += fl;
        sendSMSReply(SenderNum, sms);					// SMS non reconnu
      }
      else {
        if (!efface) {
          bool ok = false;
          if (indexreplace == 0) {
            ok = modem.addPhonebookEntry(newnumero, newnom); //ecriture dans PhoneBook
          } else {
            ok = modem.addPhonebookEntry(newnumero, newnom, indexreplace); //ecriture dans PhoneBook
          }
          Alarm.delay(100);
          if (ok) {
            message += "Nouvelle entree Phone Book";
          } else {
            message += "Erreur entree Phone Book";
          }
        }
        sendSMSReply(SenderNum, sms);
      }
    }
    else if (textesms.indexOf("LST") == 0) {	//	Liste des Num Tel
      int nligne = modem.ListPhonebook(1, 10);
      for (int idx = 1; idx < nligne + 1; idx ++) {
        message += String(idx) + ":";
        message += modem.readPhonebookEntry(idx).number;
        message += ",";
        message += modem.readPhonebookEntry(idx).text;
        message += fl;
      }
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("ETAT")) == 0 || textesms.indexOf(F("ST")) == 0) {			// "ETAT? de l'installation"
      generationMessage();
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("SYS")) == 0) { //	SYS? Etat Systeme
      byte n = modem.getRegistrationStatus();
      if (n == 5) {														// Operateur roaming
        message += F("rmg, ");								// roaming
        message += modem.getOperator() + fl;
      }
      else {
        message += modem.getOperator() + fl;  // Operateur
      }
      message += read_RSSI() + fl;
      message += "V SIM808 = ";
      message	+= String(modem.getBattVoltage());
      message += " mV, ";
      message += String(modem.getBattPercent()) + "%" + fl;

      if (getGPSdata()) {
        message += "GPS OK" + fl;
      } else {
        message += "GPS KO" + fl;
      }

      if (modem.isGprsConnected()) {
        message += "GPRS connected" + fl;
      } else {
        message += "GPRS KO" + fl;
      }

      message += ("Ver:") + String(ver) + fl;

      message += F("Valim generale = ");
      message += String(VBatterieProc) + "mV";

      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("TIMERLENT")) == 0) { //	Timer lent
      if ((textesms.indexOf(char(61))) == 9) {
        int i = textesms.substring(10).toInt();
        if (i > 9 && i <= 3601) {								//	ok si entre 10 et 3600
          config.tlent = i;
          sauvConfig();													// sauvegarde en EEPROM
          Alarm.disable(send);
          send = Alarm.timerRepeat(config.tlent, senddata); // send data
          Alarm.enable(send);
        }
      }
      message += F("Timer Lent = ");
      message += String(config.tlent);
      message += fl;
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("TIMERRAPIDE")) == 0) { //	Timer rapide
      if ((textesms.indexOf(char(61))) == 11) {
        int i = textesms.substring(12).toInt();
        if (i > 4 && i <= 3601) {								//	ok si entre 5 et 3600
          config.trapide = i;
          sauvConfig();													// sauvegarde en EEPROM
          Alarm.disable(send);
          send = Alarm.timerRepeat(config.trapide, senddata); // send data
          Alarm.enable(send);
        }
      }
      message += F("Timer Rapide = ");
      message += String(config.trapide);
      message += fl;
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("VITESSEMINI")) == 0) { //	Vitesse mini
      if ((textesms.indexOf(char(61))) == 11) {
        int i = textesms.substring(12).toInt();
        if (i > 0 && i <= 21) {								//	ok si entre 1 et 20
          config.vtransition = i;
          sauvConfig();													// sauvegarde en EEPROM
        }
      }
      message += F("Vitesse mini = ");
      message += String(config.vtransition);
      message += fl;
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("PARAM")) > 0) {
      /*
        {"param":{"timerlent":600,"timerrapide":15,"vitessemini":2}}
      */
      bool erreur = false;
      // Serial.print("position X:"),Serial.println(textesms.substring(7, 8));
      // Serial.print("position ::"),Serial.println(textesms.substring(8, 9));
      if (textesms.substring(8, 9) == ":") {
        // json en reception sans lumlut
        DynamicJsonDocument doc(104);
        DeserializationError err = deserializeJson(doc, textesms);
        if (err) {
          erreur = true;
        }
        else {
          // Serial.print(F("Deserialization succeeded"));
          JsonObject param = doc["PARAM"];
          config.tlent = param["TIMERLENT"];
          config.trapide = param["TIMERRAPIDE"];
          config.vtransition = param["VITESSEMINI"];
          sauvConfig();
        }
      }
      else {
        erreur = true;
      }
      if (!erreur) {
        // calculer taille https://arduinojson.org/v6/assistant/
        DynamicJsonDocument doc(104);
        JsonObject param = doc.createNestedObject("param");
        param["timerlent"] = config.tlent;
        param["timerrapide"] = config.trapide;
        param["vitessemini"] = config.vtransition;
        String jsonbidon;
        serializeJson(doc, jsonbidon);
        // serializeJson(doc, Serial);
        message += jsonbidon;
      }
      else {
        message += "erreur json";
      }
      message += fl;
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("MQTTDATA")) == 0) {
      // Parametres MQTTDATA=Serveur:User:Pass:Topic
      // MQTTDATA=philippeco.hopto.org:TpcfUser:hU4zHox1iHCDHM2:localisation:1883
      bool valid = false;
      if ((textesms.indexOf(char(61))) == 8) {
        byte w = textesms.indexOf(":");
        byte x = textesms.indexOf(":", w + 1);
        byte y = textesms.indexOf(":", x + 1);
        byte z = textesms.indexOf(":", y + 1);
        byte zz = textesms.length();
        // Serial.printf("%d:%d:%d:%d\n",w,x,y,z);
        Serial.printf("%d:%d:%d:%d:%d\n",w-9,x-w-1,y-x-1,z-y-1,zz-z-1);
        if(textesms.substring(z+1, zz).toInt() > 0){// Port > 0
          if((w-9)<25 && (x-w-1)<11 && (y-x-1)<16 && (z-y-1)<16){
            String sserveur = textesms.substring(9, w);
            String suser = textesms.substring(w+1, x);
            String spass = textesms.substring(x+1, y);
            String stopic = textesms.substring(y+1, z);
            Serial.print("server:"),Serial.println(sserveur);
            Serial.print("user:"),Serial.println(suser);
            Serial.print("pass:"),Serial.println(spass);
            Serial.print("topic:"),Serial.println(stopic);
            Serial.print("port:"),Serial.println(textesms.substring(z+1, zz).toInt());
            sserveur.toCharArray(config.mqttServer,(sserveur.length()+1));
            suser.toCharArray(config.mqttUserName,(suser.length()+1));
            spass.toCharArray(config.mqttPass,(spass.length()+1));
            stopic.toCharArray(config.writeTopic,(stopic.length()+1));
            config.mqttPort = textesms.substring(z+1, zz).toInt();
            valid = true;
          }
        }
        if (valid) {
          sauvConfig();													// sauvegarde en EEPROM
          message += "Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant";
          message += fl;
        } else {
          message += "Erreur format";
          message += fl;
        }
      }
      message += F("Parametres MQTT :");
      message += fl;
      message += "Serveur:"+String(config.mqttServer)+fl;
      message += "User:"+String(config.mqttUserName)+fl;
      message += "Pass:"+String(config.mqttPass)+fl;
      message += "Topic:"+String(config.writeTopic)+fl;
      message += "Port:"+String(config.mqttPort)+fl;
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("GPRSDATA")) == 0) {
      // Parametres GPRSDATA = APN:USER:PASS
      bool valid = false;
      if ((textesms.indexOf(char(61))) == 8) {
        byte x = textesms.indexOf(":");
        byte y = textesms.indexOf(":", x + 1);
        byte z = textesms.length();
        Serial.printf("%d:%d:%d\n",x-9,y-x-1,z-y-1);
        if ((x-9)<11 && (y-x-1)<11 && (z-y-1)<11) {
          String sapn = textesms.substring(9, x);
          String suser = textesms.substring(x + 1, y);
          String spass = textesms.substring(y + 1, z);
          Serial.println(sapn);
          Serial.println(suser);
          Serial.println(spass);
          sapn.toCharArray(config.apn, (sapn.length() + 1));
          suser.toCharArray(config.gprsUser, (suser.length() + 1));
          spass.toCharArray(config.gprsPass, (spass.length() + 1));
          valid = true;
        }
        if (valid) {
          sauvConfig();													// sauvegarde en EEPROM
          message += "Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant";
          message += fl;
        } else {
          message += "Erreur format";
          message += fl;
        }
      }
      message += F("Parametres GPRS APN:USER:PASS =");
      message += fl;
      message += String(config.apn);
      message += ":";
      message += String(config.gprsUser);
      message += ":";
      message += String(config.gprsPass);
      message += fl;
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("IMEI")) > -1) {
      message += F("IMEI = ");
      message += modem.getIMEI();
      message += fl;
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("TIMEOUTWIFI")) > -1) { // Parametre Arret Wifi
      if (textesms.indexOf(char(61)) == 11) {
        int n = textesms.substring(12, textesms.length()).toInt();
        if (n > 9 && n < 3601) {
          config.timeoutWifi = n;
          sauvConfig();														// sauvegarde en EEPROM
        }
      }
      message += F("TimeOut Wifi (s) = ");
      message += config.timeoutWifi;
      message += fl;
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("Wifi")) == 0) { // demande connexion Wifi
      byte pos1 = textesms.indexOf(char(44));//","
      byte pos2 = textesms.indexOf(char(44), pos1 + 1);
      String ssids = textesms.substring(pos1 + 1, pos2);
      String pwds  = textesms.substring(pos2 + 1, textesms.length());
      char ssid[20];
      char pwd[20];
      ssids.toCharArray(ssid, ssids.length() + 1);
      ssids.toCharArray(ssid, ssids.length() + 1);
      pwds.toCharArray(pwd, pwds.length() + 1);
      Serial.print("ssid:"), Serial.println(ssid);
      ConnexionWifi(ssid, pwd, SenderNum, sms, index); // sms reponse sera généré par routine
    }
    else if (textesms.indexOf(F("WIFIOFF")) > -1) { // Arret Wifi
      message += F("Wifi off");
      message += fl;
      sendSMSReply(SenderNum, sms);
      WifiOff();
    }
    else if (textesms.indexOf(F("POSITION")) == 0) {	// demande position
      // on lance la demande au GPS
      if (getGPSdata()) {
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
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms == F("RST")) {               // demande RESET
      message += F("Le systeme va etre relance");  // apres envoie du SMS!
      message += fl;
      FlagReset = true;                            // reset prochaine boucle
      sendSMSReply(SenderNum, sms);
    }
    else if (!sms && textesms.indexOf(F("CALIBRATION=")) == 0) {
      /* Uniquement =.2 pour cette application */

      /* 	Mode calibration mesure tension
          Seulement en mode serie local
          recoit message "CALIBRATION=.X"
          entrer mode calibration
          Selection de la tenssion à calibrer X
          X = 1 TensionBatterie : PinBattSol : CoeffTension1
          X = 2 VBatterieProc : PinBattProc : CoeffTension2
          X = 3 VUSB : PinBattUSB : CoeffTension3
          X = 4 Tension24 : Pin24V : CoeffTension4
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
      String Sbidon = textesms.substring(12, 16); // texte apres =
      //Serial.print(F("Sbidon=")),Serial.print(Sbidon),Serial.print(char(44)),Serial.println(Sbidon.length());
      long tension = 0;
      if (Sbidon.substring(0, 1) == "." && Sbidon.length() > 1) { // debut mode cal
        // if (Sbidon.substring(1, 2) == "1" ) {
        // M = 1;
        // P = PinBattSol;
        // coef = CoeffTension[0];
        // }
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
      else if (FlagCalibration && Sbidon.substring(0, 4).toInt() > 0 && Sbidon.substring(0, 4).toInt() <= 8000) {
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
      sendSMSReply(SenderNum, sms);
    }
    else {
      message += F("message non reconnu");
      sendSMSReply(SenderNum, sms);
    }
  }
  else {
    Serial.println("sendername non reconnu");
  }

sortir:
  if (sms) {
    EffaceSMS(index);
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
    Alarm.disable(send);
    send = Alarm.timerRepeat(config.tlent, senddata);
    Alarm.enable(send);
    senddata();
    Serial.print("cadence  = "), Serial.println(config.tlent);
  }
  else if (Accu > 0 && lastroule == false) {
    lastroule = true;
    Alarm.disable(send);
    send = Alarm.timerRepeat(config.trapide, senddata);
    Alarm.enable(send);
    senddata();
    Serial.print("cadence  = "), Serial.println(config.trapide);
  }
  // Serial.print("speed = "), Serial.println(speed);
  // Serial.print("Accu  = "), Serial.println(Accu);
}
//---------------------------------------------------------------------------
bool getGPSdata() {

  int alt, vsat, usat;
  bool fix = false;
  fix = modem.getGPS(&lat, &lon, &speed, &alt, &course, &vsat, &usat);
  if (!fix) return fix; // sortir si fix false
  // int year, month, day, hour, minute, second = 0;
  // modem.getGPSTime(&year, &month, &day, &hour, &minute, &second);



  fieldsToPublish[0] = config.writeTopic;//"localisation";//"date";
  // fieldsToPublish[1] = "2";//"id";
  // fieldsToPublish[2] = "3";//"mylat";
  // fieldsToPublish[3] = "4";//"mylon";
  // fieldsToPublish[4] = "5";//"speed";
  // fieldsToPublish[5] = "6";//"course";

  // pour test
  char bidon[20];
  sprintf(bidon, "%04d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
  // Serial.println(bidon);//17/01/2020 09:10:11
  char charlon[12];
  sprintf(charlon, "%02.8lf", lon);
  // Serial.println(charlon);
  char charlat[12];
  sprintf(charlat, "%02.8lf", lat);
  // Serial.println(charlat);
  char charspeed[6];
  sprintf(charspeed, "%02.2lf", speed);
  // Serial.println(charspeed);
  char charcourse[7];
  sprintf(charcourse, "%03.2lf", course);
  // Serial.println(charcourse);

  // dataToPublish[0] = bidon;
  // dataToPublish[1] = Id;
  // dataToPublish[2] = charlat;
  // dataToPublish[3] = charlon;
  // dataToPublish[4] = charspeed;
  // dataToPublish[5] = charcourse;

  // {"Id":"BB63000","date_tx_sms":"2020-02-10 15:15:15","lat":42.123456,"lon":2.123456,"speed":25.2,"course":180}
  // JsonDoc.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

  DynamicJsonDocument JsonDoc (200);
  JsonDoc["Id"] = config.Idchar;
  JsonDoc["date_tx_sms"] = bidon;
  JsonDoc["lat"] = String(lat, 8);
  JsonDoc["lon"] = String(lon, 8);
  // JsonDoc["speed"] = String(speed, 1);
  // JsonDoc["course"] = String(course,0);

  char JSONmessageBuffer[200];
  serializeJson(JsonDoc, JSONmessageBuffer);
  // serializeJson(JsonDoc, Serial);

  dataToPublish[0] = JSONmessageBuffer;
  // Serial.println(JSONmessageBuffer);

  // mqtt.publish(pub_topic, JSONmessageBuffer);
  return fix;
}

//---------------------------------------------------------------------------
int mqttSubscriptionCallback( char* topic, byte* payload, unsigned int mesLength ) {
  /* 6) Use the mqttSubscriptionCallback function to handle incoming MQTT messages.
    The program runs smoother if the main loop performs the processing steps instead of the callback.
    In this function, use flags to cause changes in the main loop. */
  /**
     Process messages received from subscribed channel via MQTT broker.
       topic - Subscription topic for message.
       payload - Field to subscribe to. Value 0 means subscribe to all fields.
       mesLength - Message length.
  */
  char p[mesLength + 1];
  memcpy( p, payload, mesLength );
  p[mesLength] = NULL;
  Serial.print( "Answer: " );
  Serial.println( String(p) );
  // servo_pos = atoi( p );
  // changeFlag = 1;
}
//---------------------------------------------------------------------------
void mqttConnect() {
  // 7) Use the MQTTConnect function to set up and maintain a connection to the MQTT. This function generates a random client ID for connecting to the ThingSpeak MQTT server.

  // char clientID[ 9 ];

  // Loop until connected.
  while ( !mqttClient.connected()) {
    // getID(clientID, 8);
    // Connect to the MQTT broker.
    Serial.print("Attempting MQTT connection...");
    if ( mqttClient.connect(config.Idchar, config.mqttUserName, config.mqttPass)) {
      Serial.println( "Connected with Client ID:  " + String(config.Idchar) + " User " + String(config.mqttUserName) + " Pwd " + String(config.mqttPass));
    } else {
      Serial.print( "failed, rc = " );
      // See https://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
      Serial.print( mqttClient.state() );
      Serial.println( " Will try again in 5 seconds" );
      delay( 5000 );
    }
  }
}
//---------------------------------------------------------------------------
// void getID(char clientID[], int idLength) {
//8) Use getID to generate a random client ID for connection to the MQTT server.

// /*
// Build a random client ID.
// clientID - Character array for output
// idLength - Length of clientID (actual length is one character longer for NULL)
// */
// static const char alphanum[] = "0123456789"
// "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
// "abcdefghijklmnopqrstuvwxyz"; // For random generation of the client ID.


// for (int i = 0; i < idLength ; i++) { // Generate client ID.
// clientID[ i ] = alphanum[ random( 51 ) ];
// }
// clientID[ idLength ] = '\0';
// }
//---------------------------------------------------------------------------
int mqttSubscribe( long subChannelID, int field, char* readKey, int unsubSub ) {
  // 9) Use mqttSubscribe to receive updates from the LED control field. In this example, you subscribe to a field, but you can also use this function to subscribe to the whole channel feed. Call the function with field = 0 to subscribe to the whole feed.

  /**
     Subscribe to fields of a channel.
       subChannelID - Channel to subscribe to.
       field - Field to subscribe to. Value 0 means subscribe to all fields.
       readKey - Read API key for the subscribe channel.
       unSub - Set to 1 for unsubscribe.
  */

  String myTopic;

  // There is no field zero, so if field 0 is sent to subscribe to, then subscribe to the whole channel feed.
  if (field == 0) {
    myTopic = "channels/" + String( subChannelID ) + "/subscribe/json/" + String( readKey );
  }
  else {
    myTopic = "channels/" + String( subChannelID ) + "/subscribe/fields/field" + String( field ) + "/" + String( readKey );
  }

  Serial.println( "Subscribing to " + myTopic );
  Serial.println( "State= " + String( mqttClient.state() ) );

  if ( unsubSub == 1 ) {
    return mqttClient.unsubscribe(myTopic.c_str());
  }
  return mqttClient.subscribe( myTopic.c_str() , 0 );

}
//---------------------------------------------------------------------------
void mqttPublish(char* pubChannelID, String dataArray[], String fieldArray[]) {
  int idxdata = 0;
  String dataString = "";
  while (idxdata < nbrdata) {
    dataString +=  String(dataArray[idxdata]);
    idxdata++;
  }
  Serial.println(pubChannelID);
  Serial.println( dataString );

  bool rep = mqttClient.publish(pubChannelID, dataString.c_str());
  Serial.print("publication :"), Serial.println(rep);
}
//---------------------------------------------------------------------------
void mqttPublishThingspeak(long pubChannelID, char* pubWriteAPIKey, String dataArray[], String fieldArray[]) {
  // 11) Use the mqttPublish function to send your Wi-Fi RSSI data to a ThingSpeak channel.

  /**
     Publish to a channel
       pubChannelID - Channel to publish to.
       pubWriteAPIKey - Write API key for the channel to publish to.
       dataArray - Binary array indicating which fields to publish to, starting with field 1.
       fieldArray - Array of values to publish, starting with field 1.
  */

  int idxdata = 0;
  String dataString = "";
  while (idxdata < nbrdata) {
    dataString += "&" + fieldArray[idxdata] + "=" + String(dataArray[idxdata]);
    idxdata++;
  }

  Serial.println( dataString );

  // Create a topic string and publish data to ThingSpeak channel feed.
  String topicString = "channels/" + String( pubChannelID ) + "/publish/" + String( pubWriteAPIKey );
  mqttClient.publish(topicString.c_str(), dataString.c_str());
  Serial.println("Published to channel " + String(pubChannelID));
}
//---------------------------------------------------------------------------
void generationMessage() {
  /* Generation du message etat/alarme général */

  messageId();
  if (!true) {
    message += F("--KO--------KO--");
  }
  else {
    message += F("-------OK-------");
  }
  message += fl;
  message += F("Batterie : ");				//"Alarme Batterie : "
  if (false) {
    message += F("Alarme, ");
    // message += fl;// V2-15
  }
  else {
    message += F("OK, ");
    // message += fl;// V2-15
  }
  message += fl;
  message += "Valim generale = ";
  message += String(VBatterieProc) + "mV";
  message += fl;
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
  read_adc(PinBattProc); // lecture des adc
}
//---------------------------------------------------------------------------
void read_adc(int pin2) {
  // http://www.f4grx.net/algo-comment-calculer-une-moyenne-glissante-sur-un-microcontroleur-a-faibles-ressources/
  static int plus_ancien = 0;
  //acquisition
  int sample[5];
  for (byte i = 0; i < 5; i++) {
    // if (i == 0)sample[i] = moyenneAnalogique(pin1);
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
      err = modem.emptySMSBuffer(); // dell all sms
      Serial.print(F("resultat delall Sms "));	Serial.println(err);
      break;
    }
  } while (!err);
}
//---------------------------------------------------------------------------
void ConnexionWifi(char* ssid, char* pwd, String number, bool sms, int index) {
  mqttClient.disconnect();
  modem.gprsDisconnect();
  messageId();
  Serial.print(F("connexion Wifi:")), Serial.print(ssid), Serial.print(char(44)), Serial.println(pwd);
  String ip;
  WiFi.begin(ssid, pwd);
  // WiFi.mode(WIFI_STA);
  byte timeout = 0;
  bool error = false;

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
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
  sendSMSReply(number, sms);

  if (sms) { // suppression du SMS
    /* Obligatoire ici si non bouclage au redemarrage apres timeoutwifi
      ou OTA sms demande Wifi toujours present */
    EffaceSMS(index);
  }
  debut = millis();
  if (!error) {
    /* boucle permettant de faire une mise à jour OTA et serveur, avec un timeout en cas de blocage */
    unsigned long timeout = millis();
    while (millis() - timeout < config.timeoutWifi * 1000) {
      // if(WiFi.status() != WL_CONNECTED) break; // wifi a été coupé on sort
      ArduinoOTA.handle();
      server_wifi.handleClient(); // Listen for client connections
      delay(1);
    }
    WifiOff();
  }
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
  // GPIO13 to RS reset hard
  pinMode(PinReset, OUTPUT);
  digitalWrite(PinReset, LOW);
}
//---------------------------------------------------------------------------
void OuvrirFichierCalibration() { // Lecture fichier calibration

  if (SPIFFS.exists(filecalibration)) {
    File f = SPIFFS.open(filecalibration, "r");
    for (int i = 0; i < 4; i++) { //Read
      String s = f.readStringUntil('\n');
      CoeffTension[i] = s.toFloat();
    }
    f.close();
  }
  else {
    Serial.print(F("Creating Data File:")), Serial.println(filecalibration); // valeur par defaut
    CoeffTension[0] = CoeffTensionDefaut;
    CoeffTension[1] = CoeffTensionDefaut;
    CoeffTension[2] = CoeffTensionDefaut;
    CoeffTension[3] = CoeffTensionDefaut;
    Recordcalib();
  }
  Serial.print(F("Coeff T Batterie = ")), Serial.print(CoeffTension[0]);
  Serial.print(F(" Coeff T Proc = "))	  , Serial.print(CoeffTension[1]);
  Serial.print(F(" Coeff T VUSB = "))		, Serial.print(CoeffTension[2]);
  Serial.print(F(" Coeff T 24V = "))		, Serial.println(CoeffTension[3]);

}
//---------------------------------------------------------------------------
void Recordcalib() { // enregistrer fichier calibration en SPIFFS
  // Serial.print(F("Coeff T Batterie = ")),Serial.println(CoeffTension1);
  // Serial.print(F("Coeff T Proc = "))	  ,Serial.println(CoeffTension2);
  // Serial.print(F("Coeff T VUSB = "))		,Serial.println(CoeffTension3);
  File f = SPIFFS.open(filecalibration, "w");
  f.println(CoeffTension[0]);
  f.println(CoeffTension[1]);
  f.println(CoeffTension[2]);
  f.println(CoeffTension[3]);
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

  webpage += F("<tr>");
  webpage += F("<td>MQTT Topic</td>");
  webpage += F("<td>");	webpage += String(config.writeTopic);	webpage += F("</td>");
  webpage += F("</tr>");

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
  // if (gsm) {
  int nligne = modem.ListPhonebook(1, 10);
  for (int idx = 1; idx < nligne + 1; idx ++) {
    webpage += F("<tr>");
    webpage += F("<td>"); webpage += String(modem.readPhonebookEntry(idx).text); webpage += F("</td>");
    webpage += F("<td>"); webpage += String(modem.readPhonebookEntry(idx).number); webpage += F("</td>");
    webpage += F("</tr>");
  }
  webpage += F("</table><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
  // }
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
void PrintEEPROM() {
  Serial.print(F("Version = "))                 , Serial.println(ver);
  Serial.print(F("ID = "))                      , Serial.println(config.Idchar);
  Serial.print(F("magic = "))                   , Serial.println(config.magic);
  Serial.print(F("apn = "))                     , Serial.println(config.apn);
  Serial.print(F("gprsUser = "))                , Serial.println(config.gprsUser);
  Serial.print(F("gprspass = "))                , Serial.println(config.gprsPass);
  Serial.print(F("mqttServeur = "))             , Serial.println(config.mqttServer);
  Serial.print(F("mqttUserName = "))            , Serial.println(config.mqttUserName);
  Serial.print(F("mqttPass = "))                , Serial.println(config.mqttPass);
  Serial.print(F("writeTopic = "))              , Serial.println(config.writeTopic);
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
  dist_calc *= 6366000.0; //Convertion en metres 6371000

  //Serial.println(dist_calc);
  return dist_calc;
}
//---------------------------------------------------------------------------
void sauvConfig() { // sauve configuration en EEPROM
  EEPROM.begin(512);
  EEPROM.put(confign, config);
  EEPROM.commit();
  EEPROM.end();
}
//---------------------------------------------------------------------------
void sendSMSReply(String num , boolean sms) { //char *cmd  String message
  // si sms=true Envoie SMS, sinon Serialprint seulement
  if (sms) {
    if (!modem.sendSMS(num, message)) {
      Serial.println(F("Envoi SMS Failed"));
    } else {
      Serial.println(F("SMS Sent OK"));
    }
  }
  Serial.print (F("Message (long) = ")), Serial.println(message.length());
  Serial.println(F("****************************"));
  Serial.println(message);
  Serial.println(F("****************************"));
}
//---------------------------------------------------------------------------
bool MajHeure() {
  int alt, vsat, usat;
  bool fix = false;
  fix = modem.getGPS(&lat, &lon, &speed, &alt, &course, &vsat, &usat);
  if (fix) {
    int yr, mnth, dy, hr, mn, sec = 0;
    modem.getGPSTime(&yr, &mnth, &dy, &hr, &mn, &sec);
    setTime(hr, mn, sec, dy, mnth, yr);
    Serial.println(displayTime());
    if (!HeureEte()) {
      hr ++;
      setTime(hr, mn, sec, dy, mnth, yr);
    }
    Serial.println(displayTime());
  }
  else {
    return fix; // sortir si fix false
  }
}
//---------------------------------------------------------------------------
boolean HeureEte() {
  // return true en été, false en hiver (1=dimanche)
  boolean Hete = false;
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
String read_RSSI() {	// lire valeur RSSI et remplir message

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
    interpretemessage(serialmessage);
    newData = false;
    serialmessage = "";
  }
}

void interpretemessage(String demande) {
  String bidons;
  //demande.toUpperCase();
  if (demande.indexOf(char(61)) == 0) {
    bidons = demande.substring(1);
    // bidons.toCharArray(replybuffer, bidons.length() + 1);
    // speed = String(replybuffer).toInt();
    messagetest = bidons;
    traite_sms(99);//	traitement SMS en mode test local
  }
}
