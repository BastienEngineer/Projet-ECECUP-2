#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>
#include <SimpleTimer.h>
SimpleTimer timer; // Timer pour échantillonnage

#define RightMotorSpeed 5 // Moteur A+
#define RightMotorDir   0 // Moteur A-
#define LeftMotorSpeed  4 // Branché au moteur B+
#define LeftMotorDir    2 // Branché au moteur B- (shield motor)
#define encoder 0 // Pin fourche optique
#define encoder3 2


volatile byte counterA = 0;
volatile byte counterB = 0;
long rpm1 = 0.0, rpm2=0.0;
unsigned long time1 = 0,time2=0;

double tempsEcoule=100;
double erreur=0;
double erreurPrecedente;
double mesure,consigne=3.3,consigneCorr;
double erreurCumulee, variationErreur;
float kp = 200.0; // Coefficient proportionnel
float ki = 4.0; // Coefficient intégrateur
float kd = 100.0; // Coefficient dérivateur

 //Interruption sur tick de la codeuse 
void ICACHE_RAM_ATTR countpulse() {
  counterA++; 
 // On incrémente le nombre de tick de la codeuse
}
void ICACHE_RAM_ATTR countpulse2() {
  counterB++;
}



const char* ssid =*********";
const char* password ="*******";
const char* mqtt_server ="51.210.183.96";

const char* mqttUser = "Squad2045";
const char* mqttPassword ="******";
const int trigPin = 14;  // Trigger (emission)
const int echoPin = 12;  // Echo    (réception)
Servo servomoteur;
#define SS_PIN 13
#define RST_PIN 15
#define Max_Acces 3
byte Count_acces=0; 
byte CodeVerif=0; 
byte Code_Acces[8]={0xFAAC2A28,0xCAD83929,0xCAD83929,0xF9D460B3,0x89615BB3,0x897062B3,0x2A453A29,0x6A5C2928 }; 

MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class

// Init array that will store new NUID 
byte nuidPICC[4];


WiFiClient espClient;
PubSubClient client(espClient);

int ultrason()
{
  long duree;   // durée de l'echo
  int distance; // distance
  
  // Émission d'un signal de durée 10 microsecondes
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Écoute de l'écho 
  duree = pulseIn(echoPin, HIGH);

  // Calcul de la distance
  distance = duree*0.034/2;

 // Affichage de la distance dans le Moniteur Série
 Serial.print("Distance : ");
 Serial.print(distance);
 Serial.println("cm");
 return distance;
}

double calculPID(double mes) // Correcteur PID (asservissement)
{
  // Calcul des erreurs
  erreur = consigne - mes;
  erreurCumulee += erreur * tempsEcoule;
  variationErreur = (erreur - erreurPrecedente) / tempsEcoule;
  erreurPrecedente = erreur;
  
  // PID : calcul de la commande
  double corr = kp * erreur + ki * erreurCumulee + kd * variationErreur;
  
  // Normalisation et contrôle du moteur
  if(corr < 0) corr=0;
  else if(corr > 1023) corr = 1023;
  
  return corr;
}

void compteur(){
  if (counterA >= 20) // Si le nombre de trous dépasse 20 (= 1 tour)
 {
   rpm1=(30*1000/(millis()-time1))*counterA;
   time1 = millis();
   counterA = 0; // Répète le processus en boucle infinie
   rpm1=rpm1*0.013*6.28/60;
   Serial.println(rpm1); // Affiche la vitesse en Tr/s

 }
 if (counterB >= 20) // Si le nombre de trous dépasse 20 (= 1 tour)
 {
   rpm2=(30*1000/(millis()-time2))*counterB;
   time2 = millis();
   counterB = 0; // Répète le processus en boucle infinie
   rpm2=rpm2*0.013*6.28/60;
   Serial.print(rpm2);
   consigneCorr=calculPID(rpm2); 
   Serial.println(consigneCorr);

 }
}


byte GetAccesState(byte *CodeAcces,byte *NewCode) 
{
  byte StateAcces=0; 
  if ((CodeAcces[0]==NewCode[0])&&(CodeAcces[1]==NewCode[1])&&
  (CodeAcces[2]==NewCode[2])&& (CodeAcces[3]==NewCode[3]))
    return StateAcces=1; 
  else
    return StateAcces=0; 
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("En cours de connexion ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connecte ");
}


void callback(String topic, byte* message, unsigned int length) {
  Serial.print(" Message sur le topic : ");
  Serial.print(topic);
  Serial.print(" Message: ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  
  if(topic=="squad/squad2045"){
      if(messageTemp == "avancer"){
  digitalWrite(RightMotorDir,LOW); // avancer
 digitalWrite(LeftMotorDir,LOW);
 analogWrite(RightMotorSpeed,1023);
 analogWrite(LeftMotorSpeed,1023); // consigneCorr
 Serial.print("Avancer");
      }
      /*
      if(messageTemp=="servo")
      {
          servomoteur.write(90);
          delay(3000);
          servomoteur.write(0);
      }*/
      if(messageTemp=="rfid")
      {
          // Initialisé la boucle si aucun badge n'est présent 
  if ( !rfid.PICC_IsNewCardPresent())
    return;

  // Vérifier la présence d'un nouveau badge 
  if ( !rfid.PICC_ReadCardSerial())
    return;

  // Afffichage 
  Serial.println(F("Un badge est détecté"));

  // Enregistrer l’ID du badge (4 octets) 
  for (byte i = 0; i < 4; i++) {
    nuidPICC[i] = rfid.uid.uidByte[i];
  }

  // Vérification du code 
  CodeVerif= GetAccesState(Code_Acces,nuidPICC); 
  if (CodeVerif!=1)
  {
    Count_acces+=1;
    if(Count_acces==Max_Acces)
    {

    }

  }

  // Affichage de l'identifiant 
  Serial.println(" L'UID du tag est:");
  for (byte i = 0; i < 4; i++) 
  {
    Serial.print(nuidPICC[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Re-Init RFID
  rfid.PICC_HaltA(); // Halt PICC
  rfid.PCD_StopCrypto1(); // Stop encryption on PCD
      }
 if( messageTemp=="go")
 {
 digitalWrite(RightMotorDir,LOW); // avancer de 3 sec
 digitalWrite(LeftMotorDir,LOW);
 analogWrite(RightMotorSpeed,1023);
 analogWrite(LeftMotorSpeed,1023);
 delay(3000);
 digitalWrite(RightMotorDir,LOW); 
 digitalWrite(LeftMotorDir,LOW);
 analogWrite(RightMotorSpeed,0);
 analogWrite(LeftMotorSpeed,0); 
 }
 if(messageTemp == "ultrason")
 {
   if(ultrason()<20)
  {
 digitalWrite(RightMotorDir,LOW); 
 digitalWrite(LeftMotorDir,LOW);
 analogWrite(RightMotorSpeed,0);
 analogWrite(LeftMotorSpeed,0);
  }
  if(ultrason()>=20){
 digitalWrite(RightMotorDir,LOW); 
 digitalWrite(LeftMotorDir,LOW);
 analogWrite(RightMotorSpeed,1023);
 analogWrite(LeftMotorSpeed,1023);
  }

 }
  if(messageTemp == "tourner")
 {
  digitalWrite(RightMotorDir,LOW); 
 digitalWrite(LeftMotorDir,HIGH);
 analogWrite(RightMotorSpeed,1023);
 analogWrite(LeftMotorSpeed,0);
 delay(500);
 digitalWrite(RightMotorDir,LOW); // avancer
 digitalWrite(LeftMotorDir,LOW);
 analogWrite(RightMotorSpeed,1023);
 analogWrite(LeftMotorSpeed,1023);
  
 }
 
     if(messageTemp == "arreter"){
 digitalWrite(RightMotorDir,LOW); 
 digitalWrite(LeftMotorDir,LOW);
 analogWrite(RightMotorSpeed,0);
 analogWrite(LeftMotorSpeed,0); 
        Serial.print("Arreter");
      }
      if(messageTemp == "gauche"){
 digitalWrite(RightMotorDir,LOW); 
 digitalWrite(LeftMotorDir,HIGH);
 analogWrite(RightMotorSpeed,1023);
 analogWrite(LeftMotorSpeed,0); 
        Serial.print("Gauche");
      }
      if(messageTemp == "droite"){
 digitalWrite(RightMotorDir,HIGH); 
 digitalWrite(LeftMotorDir,LOW);
 analogWrite(RightMotorSpeed,0);
 analogWrite(LeftMotorSpeed,1023); 
        Serial.print("Droite");
      }
      if(messageTemp == "reculer"){
 digitalWrite(RightMotorDir,HIGH); 
 digitalWrite(LeftMotorDir,HIGH);
 analogWrite(RightMotorSpeed,1023);
 analogWrite(LeftMotorSpeed,1023); 
        Serial.print("Reculer");
      }

      

  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print(" Connexion du MQTT ");
    
    if (client.connect("ESP8266Client", mqttUser,mqttPassword)) {
      Serial.println(" connected ");
       client.subscribe("squad/squad2045");  
       client.publish("squad/squad2045", "Bonjour");
 
    } else {
      Serial.print("rate, rc=");
      Serial.print(client.state());
      Serial.println(" Essayons dans 5 secondes");
      delay(5000);
    }
  }
}

void setup() {
  
  Serial.begin(115200);
  pinMode(RightMotorSpeed, OUTPUT);
  pinMode(LeftMotorSpeed, OUTPUT);
  pinMode(RightMotorDir, OUTPUT);
  pinMode(LeftMotorDir, OUTPUT);
  //pinMode(encoder,INPUT); // Connecte pin de la fourche optique
  //pinMode(encoder3, INPUT);
  
  //attachInterrupt(encoder, countpulse, CHANGE); 
  //attachInterrupt(encoder3, countpulse2, CHANGE);
  // Interruption sur tick de la codeuse

  //timer.setInterval(100, compteur); 
  // Interruption pour calcul du PID et asservissement

  pinMode(trigPin, OUTPUT); // Configuration du port du Trigger comme une SORTIE
  pinMode(echoPin, INPUT);// Configuration du port de l'Echo  comme une ENTREE
  
  //servomoteur.attach(4); 
  
  SPI.begin(); 
  // Init MFRC522 
  rfid.PCD_Init(); 
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);


}

void loop() {
  //timer.run();
  //delay(10);
  if(!client.connected()) {
    reconnect();
  }
  if(!client.loop())  
  {
    client.connect("ESP8266Client",mqttUser,mqttPassword);
  }

} 
