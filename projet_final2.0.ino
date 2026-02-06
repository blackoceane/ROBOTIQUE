#include <Adafruit_seesaw.h>
#include <MeAuriga.h>
#include <Wire.h>
#include <MeGyro.h>
// Définition des capteurs infrarouges
#define NB_IR 5
#define LEDNUM 12
#define LEDPIN 44
#define RINGALLLEDS 0
Adafruit_seesaw ss;
MeRGBLed led(PORT0, LEDNUM);
MeGyro gyro(0, 0x69);
String serialMessage;

int currentLED = 0;
struct IR {
  long valeur = 1023;
  long min = 1023;
  long max = 0;
  long normalize = 1023;
};
IR sensors[NB_IR];
MeUltrasonicSensor ultraSensor(PORT_10);  // Capteur connecté au port 7

unsigned long currentTime = 0;

// Définition des moteurs et encodeurs
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

float distance = 0;

// Les tâches pouvant être faites
typedef enum Etat {
  DEBUT,
  CALIBRER,
  WAIT,
  START,
  TOURNER1,
  SUIVRE,
  RECULER,
  TOURNER2,
  STOP,

  SUIVRE2,
  TOURNER3,
  SUIVRE3,
  TOURNER4,
  SUIVRE4,
  TOURNER5,
  STOPFINAL

};

Etat etatActuel = DEBUT;
const int VITESSE = 150;
//constantes pour les broches des moteurs
//Motor Left

const int m1_pwm = 11;
const int m1_in1 = 48;  // M1 ENA
const int m1_in2 = 49;  // M1 ENB

//Motor Right
const int m2_pwm = 10;
const int m2_in1 = 46;  // M2 ENA
const int m2_in2 = 47;  // M2 ENB

// Configuration des encodeurs
void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);

  encoderRight.setPulse(9);
  encoderLeft.setPulse(9);

  encoderRight.setRatio(39.267);
  encoderLeft.setRatio(39.267);

  encoderRight.setPosPid(1.8, 0, 1.2);
  encoderLeft.setPosPid(1.8, 0, 1.2);

  encoderRight.setSpeedPid(0.18, 0, 0);
  encoderLeft.setSpeedPid(0.18, 0, 0);

  // Configuration PWM
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

// Définition des interruptions pour les encodeurs
void rightEncoderInterrupt() {
  if (digitalRead(encoderRight.getPortB()) == 0) {
    encoderRight.pulsePosMinus();
  } else {
    encoderRight.pulsePosPlus();
  }
}

void leftEncoderInterrupt() {
  if (digitalRead(encoderLeft.getPortB()) == 0) {
    encoderLeft.pulsePosMinus();
  } else {
    encoderLeft.pulsePosPlus();
  }
}



#pragma region BUZZER
MeBuzzer buzzer;

#define NOTE_B0 31
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978
#define REST 0

int tempo = 120;

int melody[] = {

  // Pink Panther theme
  // Score available at https://musescore.com/benedictsong/the-pink-panther
  // Theme by Masato Nakamura, arranged by Teddy Mason

  REST,
  2,
  REST,
  4,
  REST,
  8,
  NOTE_DS4,
  8,
  NOTE_E4,
  -4,
  REST,
  8,
  NOTE_FS4,
  8,
  NOTE_G4,
  -4,
  REST,
  8,
  NOTE_DS4,
  8,
  NOTE_E4,
  -8,
  NOTE_FS4,
  8,
  NOTE_G4,
  -8,
  NOTE_C5,
  8,
  NOTE_B4,
  -8,
  NOTE_E4,
  8,
  NOTE_G4,
  -8,
  NOTE_B4,
  8,
  NOTE_AS4,
  2,
  NOTE_A4,
  -16,
  NOTE_G4,
  -16,
  NOTE_E4,
  -16,
  NOTE_D4,
  -16,
  NOTE_E4,
  2,
  REST,
  4,
  REST,
  8,
  NOTE_DS4,
  4,

  NOTE_E4,
  -4,
  REST,
  8,
  NOTE_FS4,
  8,
  NOTE_G4,
  -4,
  REST,
  8,
  NOTE_DS4,
  8,
  NOTE_E4,
  -8,
  NOTE_FS4,
  8,
  NOTE_G4,
  -8,
  NOTE_C5,
  8,
  NOTE_B4,
  -8,
  NOTE_G4,
  8,
  NOTE_B4,
  -8,
  NOTE_E5,
  8,
  NOTE_DS5,
  1,
  NOTE_D5,
  2,
  REST,
  4,
  REST,
  8,
  NOTE_DS4,
  8,
  NOTE_E4,
  -4,
  REST,
  8,
  NOTE_FS4,
  8,
  NOTE_G4,
  -4,
  REST,
  8,
  NOTE_DS4,
  8,
  NOTE_E4,
  -8,
  NOTE_FS4,
  8,
  NOTE_G4,
  -8,
  NOTE_C5,
  8,
  NOTE_B4,
  -8,
  NOTE_E4,
  8,
  NOTE_G4,
  -8,
  NOTE_B4,
  8,

  NOTE_AS4,
  2,
  NOTE_A4,
  -16,
  NOTE_G4,
  -16,
  NOTE_E4,
  -16,
  NOTE_D4,
  -16,
  NOTE_E4,
  -4,
  REST,
  4,
  REST,
  4,
  NOTE_E5,
  -8,
  NOTE_D5,
  8,
  NOTE_B4,
  -8,
  NOTE_A4,
  8,
  NOTE_G4,
  -8,
  NOTE_E4,
  -8,
  NOTE_AS4,
  16,
  NOTE_A4,
  -8,
  NOTE_AS4,
  16,
  NOTE_A4,
  -8,
  NOTE_AS4,
  16,
  NOTE_A4,
  -8,
  NOTE_AS4,
  16,
  NOTE_A4,
  -8,
  NOTE_G4,
  -16,
  NOTE_E4,
  -16,
  NOTE_D4,
  -16,
  NOTE_E4,
  16,
  NOTE_E4,
  16,
  NOTE_E4,
  2,

};

int notes = sizeof(melody) / sizeof(melody[0]) / 2;

// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;

void jouerMusique() {
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5;  // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    buzzer.tone(melody[thisNote], noteDuration * 1.5);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    // noTone(buzzer);
  }
}



#pragma endregion



void setup() {
  ss.begin();

  gyro.begin();
  // put your setup code here, to run once:
  Serial.begin(115200);
  encoderConfig();

  // configuration des broches des moteurs en sortie
  pinMode(m1_pwm, OUTPUT);  //We have to set PWM pin as output
  pinMode(m1_in2, OUTPUT);  //Logic pins are also set as output
  pinMode(m1_in1, OUTPUT);

  pinMode(m2_pwm, OUTPUT);  //We have to set PWM pin as output
  pinMode(m2_in2, OUTPUT);  //Logic pins are also set as output
  pinMode(m2_in1, OUTPUT);


  led.setpin(LEDPIN);
  led.setColor(0, 0, 0);
  led.show();
}


void TurnInPlace(int Speed) {

  encoderLeft.setMotorPwm(Speed);
  encoderRight.setMotorPwm(Speed);
}

void Stop() {
  led.setColor(0, 0, 0);
  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);
  //Serial.println("Stop");
}
void calibrationManuel() {
  for (int i = 0; i < NB_IR; i++) {
    sensors[i].valeur = ss.analogRead(i);  // Lecture brute du capteur.
    if (sensors[i].valeur < sensors[i].min) {
      sensors[i].min = sensors[i].valeur;  // Mise à jour du minimum.
    }
    if (sensors[i].valeur > sensors[i].max) {
      sensors[i].max = sensors[i].valeur;  // Mise à jour du maximum.
    }
  }
}



void normaliser_lecture() {
  for (int i = 0; i < 5; i++) {
    sensors[i].valeur = ss.analogRead(i);
    sensors[i].normalize = ((sensors[i].valeur - sensors[i].min) * 1.0) / (sensors[i].max - sensors[i].min) * 1000.0;
  }
}

// Normalisation des lectures des capteurs
double capteurLectureNormalise(int index) {
  sensors[index].valeur = ss.analogRead(index);
  // Serial.println("// Lecture brute");
  // Serial.println(sensors[index].valeur);

  if (sensors[index].max == sensors[index].min) { return 1000; }  // Éviter division par zéro


  //Serial.println((sensors[2].valeur - sensors[2].min) * 1000.0) / ((sensors[2].max - sensors[2].min) * 1.0);
  return ((sensors[index].valeur - sensors[index].min) * 1000.0) / ((sensors[index].max - sensors[index].min) * 1.0);
}

// Calcul de la position de la ligne
double calculerPositionLigne() {
  double numerateur = 0;
  double denominateur = 0;

  for (int i = 0; i < NB_IR; i++) {
    long poids = 1000 * i;  // Poids basé sur la position du capteur
                            // Serial.println(capteurLectureNormalise(i));
    numerateur += capteurLectureNormalise(i) * poids;
    denominateur += capteurLectureNormalise(i);
  }

  //if (denominateur == 0) return 2000; // Ligne non détectée
  return numerateur / denominateur;
  // Serial.println(numerateur / denominateur);
}

// Calcul PID
double computePID(double position) {

  static double kp = 1.5, ki = 0.00, kd = 0.2;
  static double integral = 0, derivative = 0, lastError = 0;
  double error = position - 2000;
  //Serial.println("avant");
  // Serial.println(error);
  integral += error;
  derivative = error - lastError;
  lastError = error;
  double output = kp * error + ki * integral + kd * derivative;

  return output;


  //Serial.println(kp * error + ki * integral + kd * derivative);
}



void suivreLigne(double adjustment) {
  int leftMotorSpeed;
  int rightMotorSpeed;

  // int baseSpeed = 70;  // Vitesse de base
  int baseSpeed = distance < 60 ? 60 : 90;  // Ajustement de la vitesse en fonction de la distance

  leftMotorSpeed = baseSpeed - adjustment;
  rightMotorSpeed = -baseSpeed - adjustment;

  // Limiter les vitesses à une plage valide (par ex., -255 à 255)
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  encoderLeft.setMotorPwm(leftMotorSpeed);
  encoderRight.setMotorPwm(rightMotorSpeed);
  // Appliquer les vitesses aux moteurs
  //setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
}



void goStraight(int speed) {

  short firstRun = 0;
  static double zAngleGoal = 0.0;
  static double error = 0.0;
  static double previousError = 0.0;
  const double kp = 3.0;
  const double kd = 1.0;
  if (firstRun) {
    zAngleGoal = gyro.getAngleZ();
    encoderLeft.setTarPWM(speed);
    encoderRight.setTarPWM(-speed);
    return;
  }

  error = gyro.getAngleZ() - zAngleGoal;
  double output = kp * error + kd * (error - previousError);
  previousError = error;

  encoderLeft.setTarPWM(speed - output);
  encoderRight.setTarPWM(-speed - output);
}

void reculeStraight(int speed) {

  short firstRun = 0;
  static double zAngleGoal = 0.0;
  static double error = 0.0;
  static double previousError = 0.0;
  const double kp = 3.0;
  const double kd = 1.0;
  if (firstRun) {
    zAngleGoal = gyro.getAngleZ();
    encoderLeft.setTarPWM(-speed);
    encoderRight.setTarPWM(speed);
    return;
  }

  error = gyro.getAngleZ() - zAngleGoal;
  double output = kp * error + kd * (error - previousError);
  previousError = error;

  encoderLeft.setTarPWM(-speed - output);
  encoderRight.setTarPWM(speed - output);
}

// Calibration des capteurs IR
void etatCalibration() {

  static bool entrer = true;
  static double angleCible = 360.0;  // Angle cible de rotation
  static double angleInitial = 0.0;
  static double angleTotal = 0.0;
  static double anglePrecedent = 0.0;
  const int AJUSTEMENT_ANGLE = 6;

  if (entrer) {
    //Serial.println("Début de la calibration");
    gyro.begin();

    angleInitial = gyro.getAngleZ();
    anglePrecedent = angleInitial;
    //Serial.print("Angle initial : ");
    //Serial.println(angleInitial);
    entrer = false;
    angleTotal = 0.0;
  }

  double angleCourant = gyro.getAngleZ();

  // Calculer la différence d'angle en tenant compte du passage 180 à -180
  double difference = angleCourant - anglePrecedent;
  if (difference > 180) {
    difference -= 360;
  } else if (difference < -180) {
    difference += 360;
  }

  angleTotal += difference;
  anglePrecedent = angleCourant;

  // Serial.print("Angle total parcouru : ");
  // Serial.println(angleTotal);
  // Serial.print("val2 : ");
  // Serial.println(sensors[2].valeur);  // Continuer la rotation

  if (abs(angleTotal) < (angleCible - AJUSTEMENT_ANGLE)) {
    TurnInPlace(70);  // Continuer la rotation
    calibrationManuel();

  } else {
    etatActuel = WAIT;  // ou l'état que vous souhaitez après la calibration
    entrer = true;      // Réinitialiser pour la prochaine utilisation
  }
  // Mettre à jour les données du gyroscope
}


void etatWait() {

  Stop();
}

void etatStart() {
  int speed = 40;
 // Serial.println("START");
  goStraight(speed);
  normaliser_lecture();
  // Serial.println(sensors[2].normalize);  // Continuer la rotation

  // Stop();
  if (sensors[2].valeur <= 800) {
    Serial.println("bossssssssssssssss");

    etatActuel = TOURNER1;
    gyro.reset(0x69);
    gyro.resetData();
  }
}


void etatTourner1() {
  double angleCour = gyro.getAngleZ();

  static bool entrer = true;
  static double angleCible = 90.0;  // Angle cible de rotation
  static double angleInitial = 0.0;
  static double angleTotal = 0.0;
  static double anglePrecedent = 0.0;
  const int AJUSTEMENT_ANGLE = 6;

  if (entrer) {
    //Serial.println("Début de la calibration");
    gyro.begin();

    angleInitial = gyro.getAngleZ();
    anglePrecedent = angleInitial;
    //Serial.print("Angle initial : ");
    //Serial.println(angleInitial);
    entrer = false;
    angleTotal = 0.0;
  }

  double angleCourant = gyro.getAngleZ();

  // Calculer la différence d'angle en tenant compte du passage 180 à -180
  double difference = angleCourant - anglePrecedent;
  if (difference > 180) {
    difference -= 360;
  } else if (difference < -180) {
    difference += 360;
  }

  angleTotal += difference;
  anglePrecedent = angleCourant;
  if (abs(angleTotal) < (angleCible - AJUSTEMENT_ANGLE)) {
    Serial.print("angle TOURNE");
    Serial.println(angleCour);
    TurnInPlace(70);  // Continuer la rotation
  } else {

    etatActuel = SUIVRE;
    // etatActuel = SUIVRE;  // ou l'état que vous souhaitez après la calibration
    entrer = true;  // Réinitialiser pour la prochaine utilisation
  }
  // Mettre à jour les données du gyroscope
}

void etatSuivre() {
  // Calculer la position de la ligne
  double position = calculerPositionLigne();

  // Calculer l'ajustement PID
  double adjustment = computePID(position);

  // Ajuster les vitesses des moteurs pour suivre la ligne
  suivreLigne(adjustment);

  if (distance < 10) {
    // Stop();
    for (int i = 6; i < 11; i++) {
      led.setColor(i, 0, 0, 255);  // Bleu pour la moitié arrière
      led.show();                  // Afficher les changements
    }
    etatActuel = RECULER;
    gyro.reset(0x69);
    gyro.resetData();
  }
}
void etatReculer(unsigned long now) {
  static unsigned long debutRecule = 0;  // Garde en mémoire le temps de la dernière action.
  const int dureeRecule = 500;           // Durée en millisecondes pendant laquelle le robot avance.
  int speed = 50;
  static bool debut = true;  // Variable pour gérer la logique d'entrée (initialisation).

  if (debut) {
    debutRecule = now;
    reculeStraight(50);
    //goStraight(speed);                // Appelle une fonction externe pour faire avancer le robot.
    //lastTime = now + dureeAvance;  // Enregistre le temps de fin d'avance.
    debut = false;  // Empêche cette section d'être exécutée à chaque cycle.
  }

  // Vérifie si le délai d'avance est écoulé.
  if (now - debutRecule >= dureeRecule) {
    Serial.println("ARRETE");
    etatActuel = TOURNER2;  // Appelle une fonction externe pour arrêter le robot.
    gyro.reset(0x69);
    gyro.resetData();
    debut = true;  // Réinitialise l'état pour un prochain appel si nécessaire.
  }
}

void etatTourner2() {
  double angleCour = gyro.getAngleZ();

  static bool entrer = true;
  static double angleCible = 195.0;  // Angle cible de rotation
  static double angleInitial = 0.0;
  static double angleTotal = 0.0;
  static double anglePrecedent = 0.0;
  const int AJUSTEMENT_ANGLE = 6;

  if (entrer) {
    //Serial.println("Début de la calibration");
    gyro.begin();

    angleInitial = gyro.getAngleZ();
    anglePrecedent = angleInitial;
    //Serial.print("Angle initial : ");
    //Serial.println(angleInitial);
    entrer = false;
    angleTotal = 0.0;
  }

  double angleCourant = gyro.getAngleZ();

  // Calculer la différence d'angle en tenant compte du passage 180 à -180
  double difference = angleCourant - anglePrecedent;
  if (difference > 180) {
    difference -= 360;
  } else if (difference < -180) {
    difference += 360;
  }

  angleTotal += difference;
  anglePrecedent = angleCourant;
  if (abs(angleTotal) < (angleCible - AJUSTEMENT_ANGLE)) {
    Serial.print("angle TOURNE");
    Serial.println(angleCour);
    TurnInPlace(70);  // Continuer la rotation
  } else {

    etatActuel = SUIVRE2;
    // etatActuel = SUIVRE;  // ou l'état que vous souhaitez après la calibration
    entrer = true;  // Réinitialiser pour la prochaine utilisation
  }
  // Mettre à jour les données du gyroscope
}
void etatSuivre2() {
  // Calculer la position de la ligne
  double position = calculerPositionLigne();

  // Calculer l'ajustement PID
  double adjustment = computePID(position);

  // Ajuster les vitesses des moteurs pour suivre la ligne
  suivreLigne(adjustment);

  if (distance < 35) {
    // Stop();

    etatActuel = TOURNER3;
    gyro.reset(0x69);
    gyro.resetData();
  }
}
void etatTourner3() {
    currentTime = millis();
  double angleCour = gyro.getAngleZ();

  static bool entrer = true;
  static double angleCible = 50.0;  // Angle cible de rotation
  static double angleInitial = 0.0;
  static double angleTotal = 0.0;
  static double anglePrecedent = 0.0;
  const int AJUSTEMENT_ANGLE = 6;

  if (entrer) {
    //Serial.println("Début de la calibration");
    gyro.begin();

    angleInitial = gyro.getAngleZ();
    anglePrecedent = angleInitial;
    //Serial.print("Angle initial : ");
    //Serial.println(angleInitial);
    entrer = false;
    angleTotal = 0.0;
  }

  double angleCourant = gyro.getAngleZ();

  // Calculer la différence d'angle en tenant compte du passage 180 à -180
  double difference = angleCourant - anglePrecedent;
  if (difference > 180) {
    difference -= 360;
  } else if (difference < -180) {
    difference += 360;
  }

  angleTotal += difference;
  anglePrecedent = angleCourant;
  if (abs(angleTotal) < (angleCible - AJUSTEMENT_ANGLE)) {

    Serial.print("angle TOURNE");
    Serial.println(angleCour);
    TurnInPlace(-70);  // Continuer la rotation

  } else {
    TurnInPlace(-70);  // Continuer la rotation
    normaliser_lecture();
    // Serial.println(sensors[2].normalize);  // Continuer la rotation

    // Stop();
    if (sensors[2].valeur <= 800) {
      Serial.println("bossssssssssssssss");
     

      etatActuel = SUIVRE3;
      // ou l'état que vous souhaitez après la calibration
      entrer = true;  // Réinitialiser pour la prochaine utilisation
    }
    // Mettre à jour les données du gyroscope
  }
}
void etatSuivre3() {


  if (distance > 35) {
    // Calculer la position de la ligne
    double position = calculerPositionLigne();

    // Calculer l'ajustement PID
    double adjustment = computePID(position);

    // Ajuster les vitesses des moteurs pour suivre la ligne
    suivreLigne(adjustment);
    Serial.println("RENTRE AU CROISEMENT");
    etatActuel = SUIVRE2;
  } else if (distance <= 35) {
    // Stop();

    etatActuel = TOURNER4;
    gyro.reset(0x69);
    gyro.resetData();
  }
}
void etatStop(unsigned long now) {
  static unsigned long debutRecule = 0;  // Garde en mémoire le temps de la dernière action.
  const int dureeRecule = 250;           // Durée en millisecondes pendant laquelle le robot avance.
  int speed = 50;
  static bool debut = true;  // Variable pour gérer la logique d'entrée (initialisation).

  if (debut) {

    debutRecule = now;
    Stop();
    //goStraight(speed);                // Appelle une fonction externe pour faire avancer le robot.
    //lastTime = now + dureeAvance;  // Enregistre le temps de fin d'avance.
    debut = false;  // Empêche cette section d'être exécutée à chaque cycle.
  }
   // Vérifie si le délai d'avance est écoulé.
  if (now - debutRecule >= dureeRecule) {
    Serial.println("ARRETE");

    etatActuel = SUIVRE3;  // Appelle une fonction externe pour arrêter le robot.
    gyro.reset(0x69);
    gyro.resetData();
    debut = true;  // Réinitialise l'état pour un prochain appel si nécessaire.
  }
}

void etatTourner4() {
  
  double angleCour = gyro.getAngleZ();

  static bool entrer = true;
  static double angleCible = 100.0;  // Angle cible de rotation
  static double angleInitial = 0.0;
  static double angleTotal = 0.0;
  static double anglePrecedent = 0.0;
  const int AJUSTEMENT_ANGLE = 6;

  if (entrer) {
    //Serial.println("Début de la calibration");
    gyro.begin();

    angleInitial = gyro.getAngleZ();
    anglePrecedent = angleInitial;
    //Serial.print("Angle initial : ");
    //Serial.println(angleInitial);
    entrer = false;
    angleTotal = 0.0;
  }

  double angleCourant = gyro.getAngleZ();

  // Calculer la différence d'angle en tenant compte du passage 180 à -180
  double difference = angleCourant - anglePrecedent;
  if (difference > 180) {
    difference -= 360;
  } else if (difference < -180) {
    difference += 360;
  }

  angleTotal += difference;
  anglePrecedent = angleCourant;
  if (abs(angleTotal) < (angleCible - AJUSTEMENT_ANGLE)) {
    Serial.print("angle TOURNE");
    Serial.println(angleCour);
    TurnInPlace(80);  // Continuer la rotation
  } else {
    TurnInPlace(80);  // Continuer la rotation
    normaliser_lecture();
    // Serial.println(sensors[2].normalize);  // Continuer la rotation

    // Stop();
    if (sensors[2].valeur <= 800) {
      Serial.println("bossssssssssssssss");


      etatActuel = SUIVRE4;
      // ou l'état que vous souhaitez après la calibration
      entrer = true;  // Réinitialiser pour la prochaine utilisation
    }
  }
  // Mettre à jour les données du gyroscope
}
void etatSuivre4() {
  if (distance > 35) {
    Serial.println("J AI FAIT MON TOURNER A GAUCHE ET A DROITE MAINTENANT JE RENTRE ");
    // Calculer la position de la ligne
    double position = calculerPositionLigne();

    // Calculer l'ajustement PID
    double adjustment = computePID(position);

    // Ajuster les vitesses des moteurs pour suivre la ligne
    suivreLigne(adjustment);
    etatActuel = SUIVRE2;
  } else if (distance <= 35) {
    // Stop();
    etatActuel = TOURNER5;
    gyro.reset(0x69);
    gyro.resetData();
  }
}
void animationTask(unsigned long cT) {
  static unsigned long previousTime = 0;
  static float j = 0, f = 0, k = 0;  // Décalages pour les couleurs
  const int rate = 50;               // Temps entre chaque mise à jour (ms)

  if (cT - previousTime < rate) return;  // Limite la vitesse d'animation
  previousTime = cT;

  for (uint8_t t = 0; t < LEDNUM; t++) {
    // Calcul des valeurs RGB en utilisant des fonctions sinusoïdales
    // Les valeurs RGB varient avec le temps pour un fade fluide
    uint8_t red = 55 + 55 * sin(j + t / 2.0);    // Fading de rouge
    uint8_t green = 55 + 55 * sin(f + t / 3.0);  // Fading de vert
    uint8_t blue = 55 + 55 * sin(k + t / 4.0);   // Fading de bleu

    led.setColorAt(t, red, green, blue);  // Applique la couleur à chaque LED
  }
  led.show();  // Rafraîchit l'affichage des LEDs

  // Incréments pour faire évoluer le fading avec le temps
  j += 0.01;
  f += 0.08;
  k += 0.06;
}
void etatTourner5() {
  double angleCour = gyro.getAngleZ();

  static bool entrer = true;
  static double angleCible = 200.0;  // Angle cible de rotation
  static double angleInitial = 0.0;
  static double angleTotal = 0.0;
  static double anglePrecedent = 0.0;
  const int AJUSTEMENT_ANGLE = 6;

  if (entrer) {
    //Serial.println("Début de la calibration");
    gyro.begin();

    angleInitial = gyro.getAngleZ();
    anglePrecedent = angleInitial;
    //Serial.print("Angle initial : ");
    //Serial.println(angleInitial);
    entrer = false;
    angleTotal = 0.0;
  }

  double angleCourant = gyro.getAngleZ();

  // Calculer la différence d'angle en tenant compte du passage 180 à -180
  double difference = angleCourant - anglePrecedent;
  if (difference > 180) {
    difference -= 360;
  } else if (difference < -180) {
    difference += 360;
  }

  angleTotal += difference;
  anglePrecedent = angleCourant;
  if (abs(angleTotal) < (angleCible - AJUSTEMENT_ANGLE)) {
    Serial.print("angle TOURNE");
    Serial.println(angleCour);
    TurnInPlace(-70);  // Continuer la rotation
  } else {

    etatActuel = STOPFINAL;
    // etatActuel = SUIVRE;  // ou l'état que vous souhaitez après la calibration
    entrer = true;  // Réinitialiser pour la prochaine utilisation
  }
  // Mettre à jour les données du gyroscope
}
void etatFinal() {
  jouerMusique();
  animationTask(currentTime);
  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);

  //Serial.println("Stop");
}
void updateLEDS(float distance) {
  if (distance <= 30) {
    for (int i = 0; i < 5; i++) led.setColor(i, 255, 0, 0);  // Rouge
  } else if (distance <= 60) {
    for (int i = 0; i < 5; i++) led.setColor(i, 255, 255, 0);  // Jaune
  } else {
    for (int i = 0; i < 5; i++) led.setColor(i, 0, 255, 0);  // Vert
  }
  for (int i = 6; i < 12; i++) led.setColor(i, 0, 0, 255);  // Bleu pour la moitié arrière
  led.show();
}
void loop() {

  currentTime = millis();
  static unsigned int lastTime = 0;
  const int rate = 100;
  if (currentTime - lastTime >= rate) {
    lastTime = currentTime;
    distance = ultraSensor.distanceCm();
  }

  switch (etatActuel) {

    case DEBUT:
      // Serial.println("DEBUT");
      // etatDebut();
      break;

    case CALIBRER:
      Serial.println("calibrer");
      etatCalibration();
      break;

    case WAIT:
      //Serial.println("calibration Automatique");
      etatWait();
      break;
    case START:
      // Serial.println("Aller");
      etatStart();
      break;

    case TOURNER1:

      //Serial.println("TOURNER1");
      etatTourner1();

      break;
    case SUIVRE:
      Serial.println("SUIVRE");
      etatSuivre();
      break;
    case RECULER:
      Serial.println("RECULER");
      etatReculer(currentTime);
      break;
    case TOURNER2:
      Serial.println("J AI DEJA LE PAQUET TOURNE");
      etatTourner2();
      break;
    case STOP:
      Stop();
      break;

    case SUIVRE2:
      updateLEDS(distance);
      Serial.println("SUIS MOI PUIS CROISEMENT");
      etatSuivre2();
      break;
    case TOURNER3:
      updateLEDS(distance);
      Serial.println("CROISEMENT TOURNE A GAUCHE");
      etatTourner3();
      break;
    case SUIVRE3:
      updateLEDS(distance);
      Serial.println("J AI TOURNER A GAUCHE");
      etatSuivre3();
      break;
    case TOURNER4:
      updateLEDS(distance);
      Serial.println("J AI D ABORD TOURNER A GAUCHE MAINTENANT A DROITE");
      etatTourner4();
      break;
    case SUIVRE4:
      updateLEDS(distance);
      Serial.println("Agauche puis a droite PUIS JE SUIT");
      etatSuivre4();
      break;
    case TOURNER5:
      updateLEDS(distance);
      Serial.println("dernier truc");
      etatTourner5();

      break;
    case STOPFINAL:
      Serial.println("FINIIIIIIIIIIIII");

      etatFinal();

      break;
  }

  encoderRight.loop();
  encoderLeft.loop();
  gyro.update();
}
void serialEvent() {
  if (!Serial.available()) return;

  serialMessage = Serial.readString();

  if (serialMessage.length() > 2) {
    // Retrait de l'entête 0xFF55
    serialMessage.remove(0, 2);
  }


  if (serialMessage.startsWith("c")) {
    etatActuel = CALIBRER;
  }
  if (serialMessage.startsWith("s")) {
    etatActuel = START;
    gyro.reset(0x69);
    gyro.resetData();
  }
}
