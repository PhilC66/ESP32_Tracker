/* Tracker GPRS MQTT */
/* Ph Corbel 31/01/2020 */
/* ESP32+Sim808
  Compilation LOLIN D32,default,80MHz, ESP32 1.0.2 (1.0.4 bugg?)
  Arduino IDE 1.8.10 :  76%,  14% sur PC
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
#include "credentials_mqtt.h"

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
String ver        = "V0-0";
int    Magique    = 14;

char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
#define nSample (1<<4)    // nSample est une puissance de 2, ici 16 (4bits)
unsigned int adc_hist[5][nSample]; // tableau stockage mesure adc, 0 Batt, 1 Proc, 2 USB, 3 24V, 5 Lum
unsigned int adc_mm[5];            // stockage pour la moyenne mobile
unsigned long debut     = 0; // pour decompteur temps wifi
unsigned long timer100  = 0; // pour timer 100ms adc
int CoeffTension[4];          // Coeff calibration Tension
int CoeffTensionDefaut = 7000;// Coefficient par defaut

byte Accu = 0; // accumulateur vitesse
float lat, lng, course, speed;

String Sbidon 		= ""; // String texte temporaire
String messagetest = "";
String message = "";
String fl = "\n";                   //  saut de ligne SMS
String Id ;                         //  Id du materiel sera lu dans EEPROM
byte confign = 0;                   //  position enregistrement config EEPROM

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

PubSubClient mqttClient(client);

// WebServer server(80);
// File UploadFile;

struct  config_t           // Structure configuration sauvée en EEPROM
{
  int     magic;           // num magique
  int     trapide;         // timer send data rapide (roulage)
  int     tlent;           // timer send data lent (arret)
  int     vtransition;     // vitesse transition arret/roulage
  char    apn[11];         // APN
  char    gprsUser[11];    // user for APN
  char    gprsPass[11];    // pass for APN
  char    Idchar[11];      // Id
} ;
config_t config;

const int nbrdata = 6;
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
    config.magic         = Magique;
    config.trapide       = 15;      // secondes
    config.tlent         = 15 * 60; // secondes
    config.vtransition   = 2;       // kmh
    String temp          = "TPCF_63000";
    String tempapn       = "sl2sfr";//"free";
    String tempUser      = "";
    String tempPass      = "";
    temp.toCharArray(config.Idchar, 11);
    tempapn.toCharArray(config.apn, 7);// tempapn.length()+1);
    tempUser.toCharArray(config.gprsUser, 1);// tempUser.length()+1);
    tempPass.toCharArray(config.gprsPass, 1);//tempPass.length()+1);
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
  Serial.println(config.apn);
  if (!modem.gprsConnect(config.apn, config.gprsUser, config.gprsPass)) {
    Serial.println(" fail");
    Alarm.delay(10000);
    return;
  }
  Serial.print(F(". success"));
  bool res = modem.isGprsConnected();
  DBG("GPRS status:", res ? "connected" : "not connected");

  String ccid = modem.getSimCCID();
  Serial.println("CCID:" + ccid);

  String imei = modem.getIMEI();
  Serial.println("IMEI:" + imei);

  String cop = modem.getOperator();
  Serial.println("Operator:" + cop);

  IPAddress local = modem.localIP();
  Serial.println("Local IP:" + local.toString());

  int csq = modem.getSignalQuality();
  Alarm.delay(200);
  Serial.println();
  Serial.println("Signal quality:" + csq);

  mqttClient.setServer( server, 1883 ); // Set the MQTT broker details.
  //// mqttClient.setCallback( mqttSubscriptionCallback );   // Set the MQTT message handler function.

  if (modem.enableGPS()) {
    Serial.println("GPS Enabled");
  }
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
    if (mqttSubscribe( readChannelID, 1, readAPIKey, 0 ) == 1 ) {
      Serial.println( " Subscribed " );
    }
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
    mqttPublish( writeChannelID, writeAPIKey, dataToPublish, fieldsToPublish );
  }
}
//---------------------------------------------------------------------------
void traite_sms(int index) {
  bool sms = true;
  String textesms;
  String SenderNum;
  String Sendername;
  if (index == 99) {
    sms = false;
    textesms = messagetest;
  } else {
    Serial.print("newmessageindex: "), Serial.println(index);
    textesms = modem.readSMS(index);
    Serial.print("sms: "), Serial.println(textesms);
    SenderNum = modem.getSenderID(index, false);
    Serial.print("sender: "), Serial.println(SenderNum);
    Sendername = modem.getSenderName(index, false);
    Serial.print("sendername: "), Serial.println(Sendername);
  }

  for (byte i = 0; i < textesms.length(); i++) {
    if ((int)textesms[i] < 0 || (int)textesms[i] > 127) { // caracteres accentués interdit
      goto sortir;
    }
  }
  if ((sms && Sendername.length() > 0) || !sms) { // emetteur reconnu dans Phone Book
    messageId();
    if (!(textesms.indexOf(F("TEL")) == 0 || textesms.indexOf(F("tel")) == 0 || textesms.indexOf(F("Tel")) == 0)) {
      textesms.toUpperCase();    // passe tout en Maj sauf si "TEL"
      textesms.replace(" ", ""); // supp tous les espaces
    }
    if (textesms.indexOf(F("TEL")) == 0
        || textesms.indexOf(F("Tel")) == 0
        || textesms.indexOf(F("tel")) == 0) { // entrer nouveau num
      /*------------ a faire -------------- */
      messageId();
      Serial.println("nouveau numero");
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms == F("LST")) {	//	Liste des Num Tel
      messageId();
      /*------------ a faire -------------- */
      Serial.println("liste numero");
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("ETAT")) == 0 || textesms.indexOf(F("ST")) == 0) {			// "ETAT? de l'installation"
      generationMessage();
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("SYS")) == 0) {					//	SYS? Etat Systeme
      messageId();
      Serial.println("sys");
      sendSMSReply(SenderNum, sms);
    }
    else if (textesms.indexOf(F("ID=")) == 0) {			//	Id= nouvel Id
      String temp = textesms.substring(3);
      if (temp.length() > 0 && temp.length() < 11) {
        Id = "";
        temp.toCharArray(config.Idchar, 11);
        sauvConfig();															// sauvegarde en EEPROM
        Id = String(config.Idchar);
        Id += fl;
      }
      messageId();
      message += F("Nouvel Id");
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
    else if (textesms.indexOf(F("PARAMGPRS")) == 0) {
      // Parametre GPRS = APN:USER:PASS
      bool valid = false;
      if ((textesms.indexOf(char(61))) == 9) {
        int x = textesms.indexOf(":");
				int y = textesms.indexOf(":",x+1);
        int z = textesms.length();
        Serial.printf("%d:%d:%d\n",x,y,z);
        
        String sapn = textesms.substring(10, x);//.c_str()
        String suser = textesms.substring(x+1,y);//.c_str() 
        String spass = textesms.substring(y+1,z);//.c_str() 
        Serial.println(sapn);
        Serial.println(suser);
        Serial.println(spass);
        
        if(x > 0 && y > 0 && x < textesms.length() && y < textesms.length()){
          valid = true;
          
        }
        if(valid){
          // sauvConfig();													// sauvegarde en EEPROM          
        }
      }
      message += F("Parametre GPRS APN:USER:PASS = ");
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
    else {
      messageId();
      message += F("message non reconnu");
      sendSMSReply(SenderNum, sms);
    }
  }
  else {
    Serial.println("sendername non reconnu");
  }

sortir:
  if (sms) {
    Serial.print("delete sms: "), Serial.println(modem.emptySMSBuffer()); //emptySMSBuffer dell all sms
  }
}
//---------------------------------------------------------------------------
void Acquisition() {

  static float lastlon = 0;
  static float lastlat = 0;
  getGPSdata();
  Serial.print("distance:"), Serial.println(calc_dist(lat, lng, lastlat, lastlon), 2);
  lastlat = lat;
  lastlon = lng;
  gereCadence();

  int index = modem.newMessageIndex(0); // verifie arrivée sms

  if (index > 0) {
    Serial.print("sms recu:"), Serial.println(index);
    traite_sms(index);
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
  fix = modem.getGPS(&lat, &lng, &speed, &alt, &course, &vsat, &usat);
  if (!fix) return fix; // sortir si fix false
  // int year, month, day, hour, minute, second = 0;
  // modem.getGPSTime(&year, &month, &day, &hour, &minute, &second);

  DynamicJsonDocument JSONencoder (300);
  JSONencoder["lat"] = String(lat, 8);
  JSONencoder["lng"] = String(lng, 8);
  JSONencoder["spped"] = String(speed, 8);
  JSONencoder["alt"] = alt;
  JSONencoder["vsat"] = vsat;
  JSONencoder["usat"] = usat;
  JSONencoder["year"] = year();
  JSONencoder["month"] = month();
  JSONencoder["day"] = day();
  JSONencoder["hour"] = hour();
  JSONencoder["minute"] = minute();
  JSONencoder["second"] = second();

  char JSONmessageBuffer[300];
  serializeJson(JSONencoder, JSONmessageBuffer);
  // Serial.println(JSONmessageBuffer);

  fieldsToPublish[0] = "1";//"date";
  fieldsToPublish[1] = "2";//"id";
  fieldsToPublish[2] = "3";//"mylat";
  fieldsToPublish[3] = "4";//"mylon";
  fieldsToPublish[4] = "5";//"speed";
  fieldsToPublish[5] = "6";//"course";

  // pour test
  char bidon[20];
  sprintf(bidon, "%02d/%02d/%04d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  // Serial.println(bidon);//17/01/2020 09:10:11
  char charlon[12];
  sprintf(charlon, "%02.8lf", lng);
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

  dataToPublish[0] = bidon;
  dataToPublish[1] = Id;
  dataToPublish[2] = charlat;
  dataToPublish[3] = charlon;
  dataToPublish[4] = charspeed;
  dataToPublish[5] = charcourse;

  // JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

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
  while ( !mqttClient.connected() ) {
    // getID(clientID, 8);
    // Connect to the MQTT broker.
    Serial.print( "Attempting MQTT connection..." );
    if ( mqttClient.connect( config.Idchar, mqttUserName, mqttPass ) ) {
      Serial.println( "Connected with Client ID:  " + String( config.Idchar ) + " User " + String( mqttUserName ) + " Pwd " + String( mqttPass ) );
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
void mqttPublish(long pubChannelID, char* pubWriteAPIKey, String dataArray[], String fieldArray[]) {
  // 11) Use the mqttPublish function to send your Wi-Fi RSSI data to a ThingSpeak channel.

  /**
     Publish to a channel
       pubChannelID - Channel to publish to.
       pubWriteAPIKey - Write API key for the channel to publish to.
       dataArray - Binary array indicating which fields to publish to, starting with field 1.
       fieldArray - Array of values to publish, starting with field 1.
  */

  int index = 0;
  String dataString = "";
  while (index < nbrdata) {
    dataString += "&" + fieldArray[index] + "=" + String(dataArray[index]);
    index++;
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
  message += "100";//String(Battpct(TensionBatterie));
  message += "%";
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
  read_adc( PinBattProc   ); // lecture des adc
}
//---------------------------------------------------------------------------
void read_adc(int pin1) {
  // http://www.f4grx.net/algo-comment-calculer-une-moyenne-glissante-sur-un-microcontroleur-a-faibles-ressources/
  static int plus_ancien = 0;
  //acquisition
  int sample[5];
  for (byte i = 0; i < 5; i++) {
    if (i == 0)sample[i] = moyenneAnalogique(pin1);
    // if (i == 1)sample[i] = moyenneAnalogique(pin2);
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
void PrintEEPROM() {
  Serial.print(F("Version = "))                 , Serial.println(ver);
  Serial.print(F("ID = "))                      , Serial.println(config.Idchar);
  Serial.print(F("magic = "))                   , Serial.println(config.magic);
  Serial.print(F("apn = "))                     , Serial.println(config.apn);
  Serial.print(F("gprsUser = "))                , Serial.println(config.gprsUser);
  Serial.print(F("gprspass = "))                , Serial.println(config.gprsPass);
  Serial.print(F("Tempo rapide = "))            , Serial.println(config.trapide);
  Serial.print(F("Tempo lente = "))             , Serial.println(config.tlent);
  Serial.print(F("Vitesse mini = "))            , Serial.println(config.vtransition);
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
  fix = modem.getGPS(&lat, &lng, &speed, &alt, &course, &vsat, &usat);
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
