#define BAUD_RATE 57600
char versionStr[] = "Split-Rail 48 volt 4-line pedalometer Pedal Power Utility Box ver. 2.4 branch:decida";

// PINS
#define GROUNDPLUS 4 // HIGH on this pin shorts pedaller's plus to ground
#define GROUNDPLUSACTIVATE true
#define GROUNDMINUS 7 // LOW on this pin shorts pedaller's minus to ground
#define GROUNDMINUSACTIVATE true
#define SWITCHDELAY 500 // wait after deactivating a transistor before the next one comes on
#define RELAYPIN 2 // relay cutoff output pin // NEVER USE 13 FOR A RELAY
#define VOLTPIN A0 // Voltage Sensor Pin
#define MINUS_VOLTPIN A1 // this pin measures MINUSRAIL voltage
#define NUM_LEDS 4 // Number of LED outputs.
const int ledPins[NUM_LEDS] = { // 24v LEDS POWERED BY PLUSRAIL
  3, 5, 6, 11}; // pin 9 and 10 are used by decida transistor banks

// levels at which each LED turns on (not including special states)
const float ledLevels[NUM_LEDS+1] = { // these refer to total system voltage
  24.0, 32.0, 40.0, 47.0, 51.3}; // the last voltage is when all the LEDs will blink

#define BRIGHTNESSVOLTAGE 24.0  // voltage at which LED brightness starts to fold back
#define BRIGHTNESSBASE 255  // maximum brightness value (255 is max value here)
int brightness = 0;  // analogWrite brightness value, updated by getVolts()
#define BRIGHTNESSFACTOR (BRIGHTNESSBASE / BRIGHTNESSVOLTAGE) / 2 // results in half PWM at double voltage
// for every volt over BRIGHTNESSVOLTAGE, pwm is reduced by BRIGHTNESSFACTOR from BRIGHTNESSBASE

int analogState[NUM_LEDS] = {0}; // stores the last analogWrite() value for each LED
                                 // so we don't analogWrite unnecessarily!

#define AVG_CYCLES 50 // average measured values over this many samples
#define DISPLAY_INTERVAL 500 // when auto-display is on, display every this many milli-seconds
#define LED_UPDATE_INTERVAL 1000
#define BLINK_PERIOD 600
#define FAST_BLINK_PERIOD 150

#define STATE_OFF 0
#define STATE_BLINK 1
#define STATE_BLINKFAST 3
#define STATE_ON 2

int ledState[NUM_LEDS] = {STATE_OFF}; // on/off/blink/fastblink state of each led

#define PLUSPEDAL 1
#define MINUSPEDAL -1
#define OPENPEDAL 0
int decision = PLUSPEDAL; // which rail should pedalpower go into?
int lastDecision = 99; // store our last decision

#define RAILFULL 0.98 // how full is a rail before we decide it is full
#define DECIDA_SWITCHTIME 500 // minimum time between switching rails to pedal

#define MINIMUM_PLUSRAIL 9 // below this voltage, pedalling only goes to plusrail
#define MINIMUM_FETVOLTAGE 8 // below this PLUSRAIL voltage, OPENPEDAL only
#define MAX_PLUSRAIL 29.7
#define MAX_MINUSRAIL 29.7
#define RELAY_HYSTERESIS 4.0 // how many volts of hysteresis to have
int relayState = STATE_OFF;

#define DANGER_PLUSRAIL 30.0 // 10*2.7=27 volt cap bank
#define DANGER_MINUSRAIL 30.0 // 9*2.7=24.3 volt cap bank
int dangerState = STATE_OFF;

int blinkState = 0;
int fastBlinkState = 0;

#define VOLTCOEFF 13.179  // larger number interprets as lower voltage

int voltsAdc = 0;
float voltsAdcAvg = 0;
float volts = 0; // averaged A0 voltage PLUSRAIL

int minusAdc = 0; // for measuring A1 voltage
float minusAvg = 0; // for measuring A1 voltage
float minusRail = 0; // averaged voltage MINUSRAIL

float plusRail = 0; // averaged voltage PLUSRAIL

// timing variables for various processes: led updates, print, blink, etc
unsigned long time = 0;
unsigned long timeFastBlink = 0;
unsigned long timeBlink = 0;
unsigned long timeDisplay = 0;
unsigned long lastDecided = 0;

void setup() {
  Serial.begin(BAUD_RATE);

  Serial.println(versionStr);

  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN,LOW);

  // init LED pins
  for(int i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i],OUTPUT);
  }

  timeDisplay = millis();
  // setPwmFrequency(9,1); // set frequency of PWM on pins 9 and 10 to 31,250 Hz for buck
  pinMode(GROUNDPLUS,OUTPUT);
  pinMode(GROUNDMINUS,OUTPUT);
}

void loop() {
  time = millis();
  getVolts();
  //  doBuck(); // adjust inverter voltage
  doSafety(); // sets relay state and sets dangerState
  if (time - lastDecided > DECIDA_SWITCHTIME) {
    doDecide(); // decide which rail needs pedallers more
    lastDecided = time; // reset time timeout
  }
  doBlink();  // blink the LEDs
  doLeds();

  if(time - timeDisplay > DISPLAY_INTERVAL){
    printDisplay();
    timeDisplay = time;
  }

}

void doDecide() {
  float plusCentage = plusRail / MAX_PLUSRAIL; // how full is plusrail?
  float minusCentage = minusRail / MAX_MINUSRAIL; // how full is minusrail?

  if (plusRail < MINIMUM_FETVOLTAGE) {
    decision = OPENPEDAL; // don't try FETs without enough voltage to close them!
    Serial.println("plusRail < MINIMUM_FETVOLTAGE  ");
  } else {
    decision = PLUSPEDAL; // default to plusrail
    if ((plusRail > MINIMUM_PLUSRAIL) && (plusCentage > minusCentage)) {
      decision = MINUSPEDAL; // pedal the minus rail now
    }
    if ((plusCentage > RAILFULL) && (minusCentage > RAILFULL)) {
      decision = OPENPEDAL; // let pedalpower charge both rails
    }
  }

  if (decision != lastDecision) {
    if (decision == MINUSPEDAL) {
      if (digitalRead(GROUNDMINUS) == GROUNDMINUSACTIVATE) {
        digitalWrite(GROUNDMINUS,!GROUNDMINUSACTIVATE); // dont ground minusrail
        delayMicroseconds(SWITCHDELAY); // wait for GROUNDMINUS to deactivate)
      }
      digitalWrite(GROUNDPLUS,GROUNDPLUSACTIVATE); // ground the plusrail
    } else if (decision == PLUSPEDAL) {
      if (digitalRead(GROUNDPLUS) == GROUNDPLUSACTIVATE) {
        digitalWrite(GROUNDPLUS,!GROUNDPLUSACTIVATE); // dont ground minusrail
        delayMicroseconds(SWITCHDELAY); // wait for GROUNDPLUS to deactivate)
      }
      digitalWrite(GROUNDMINUS,GROUNDMINUSACTIVATE); // ground minusrail
    } else if (decision == OPENPEDAL) {
      digitalWrite(GROUNDMINUS,!GROUNDMINUSACTIVATE); // dont ground minusrail
      digitalWrite(GROUNDPLUS,!GROUNDPLUSACTIVATE); // dont ground the plusrail
    }
    lastDecision = decision; // store our state for next time
  }
}

#define BUCK_CUTIN 13 // voltage above which transistors can start working
#define BUCK_CUTOUT 11 // voltage below which transistors can not function
#define BUCK_VOLTAGE 26.0 // target voltage for inverter to be supplied with
#define BUCK_VOLTPIN A1 // this pin measures inverter's MINUS TERMINAL voltage
#define BUCK_HYSTERESIS 0.75 // volts above BUCK_VOLTAGE where we start regulatin
#define BUCK_PWM_UPJUMP 0.03 // amount to raise PWM value if voltage is below BUCK_VOLTAGE
#define BUCK_PWM_DOWNJUMP 0.15 // amount to lower PWM value if voltage is too high
float buckPWM = 0; // PWM value of pin 9
int lastBuckPWM = 0; // make sure we don't call analogWrite if already set right
/*
void doBuck() {
  if (volts > BUCK_CUTIN) { // voltage is high enough to turn on transistors
    if (volts <= BUCK_VOLTAGE) { // system voltage is lower than inverter target voltage
      digitalWrite(9,HIGH); // turn transistors fully on, give full voltage to inverter
      buckPWM = 0;
    }

    if ((volts > BUCK_VOLTAGE+BUCK_HYSTERESIS) && (buckPWM == 0)) { // begin PWM action
      buckPWM = 255.0 * (1.0 - ((volts - BUCK_VOLTAGE) / BUCK_VOLTAGE)); // best guess for initial PWM value
      //      Serial.print("buckval=");
      //      Serial.println(buckPWM);
      analogWrite(9,(int) buckPWM); // actually set the thing in motion
    }

    if ((volts > BUCK_VOLTAGE) && (buckPWM != 0)) { // adjust PWM value based on results
      if (volts - voltsBuck > BUCK_VOLTAGE + BUCK_HYSTERESIS) { // inverter voltage is too high
        buckPWM -= BUCK_PWM_DOWNJUMP; // reduce PWM value to reduce inverter voltage
        if (buckPWM <= 0) {
          //          Serial.print("0");
          buckPWM = 1; // minimum PWM value
        }
        if (lastBuckPWM != (int) buckPWM) { // only if the PWM value has changed should we...
          lastBuckPWM = (int) buckPWM;
          //          Serial.print("-");
          analogWrite(9,lastBuckPWM); // actually set the PWM value
        }
      }
      if (volts - voltsBuck < BUCK_VOLTAGE) { // inverter voltage is too low
        buckPWM += BUCK_PWM_UPJUMP; // increase PWM value to raise inverter voltage
        if (buckPWM > 255.0) {
          buckPWM = 255.0;
          //          Serial.print("X");
        }
        if (lastBuckPWM != (int) buckPWM) { // only if the PWM value has changed should we...
          lastBuckPWM = (int) buckPWM;
          //          Serial.print("+");
          analogWrite(9,lastBuckPWM); // actually set the PWM value
        }
      }
    }
  }
  if (volts < BUCK_CUTOUT) { // system voltage is too low for transistors
    digitalWrite(9,LOW); // turn off transistors
  }
}
*/
void doSafety() {
  if ((plusRail > MAX_PLUSRAIL) || (minusRail > MAX_MINUSRAIL)){
    digitalWrite(RELAYPIN, HIGH);
    relayState = STATE_ON;
  }

  if (relayState == STATE_ON && (plusRail < MAX_PLUSRAIL - RELAY_HYSTERESIS) && (minusRail < MAX_MINUSRAIL - RELAY_HYSTERESIS)){
    digitalWrite(RELAYPIN, LOW);
    relayState = STATE_OFF;
  }

  if ((plusRail > DANGER_PLUSRAIL) || (minusRail > DANGER_MINUSRAIL)){
    dangerState = STATE_ON;
  }
  else {
    dangerState = STATE_OFF;
  }
}

void doBlink(){

  if (((time - timeBlink) > BLINK_PERIOD) && blinkState == 1){
    blinkState = 0;
    timeBlink = time;
  }
  else if (((time - timeBlink) > BLINK_PERIOD) && blinkState == 0){
    blinkState = 1;
    timeBlink = time;
  }


  if (((time - timeFastBlink) > FAST_BLINK_PERIOD) && fastBlinkState == 1){
    fastBlinkState = 0;
    timeFastBlink = time;
  }
  else if (((time - timeFastBlink) > FAST_BLINK_PERIOD) && fastBlinkState == 0){
    fastBlinkState = 1;
    timeFastBlink = time;
  }

}

void doLeds(){

  for(int i = 0; i < NUM_LEDS; i++) {
    if(volts >= ledLevels[i]){
      ledState[i]=STATE_ON;
    }
    else
      ledState[i]=STATE_OFF;
  }

  // if voltage is below the lowest level, blink the lowest level
  if (volts < ledLevels[0]){
    ledState[0]=STATE_BLINK;
  }

  // turn off first x levels if voltage is above 3rd level
  if(volts > ledLevels[1]){
    ledState[0] = STATE_OFF;
//    ledState[1] = STATE_OFF;
  }

  if (dangerState){ // dangerState means one of the rails is too high
    for(int i = 0; i < NUM_LEDS; i++) {
      ledState[i] = STATE_BLINKFAST;
    }
  }

  if (volts >= ledLevels[NUM_LEDS]) {// if at the top voltage level, blink last LEDS fast
    ledState[NUM_LEDS-1] = STATE_BLINKFAST; // last set of LEDs
  }

  // loop through each led and turn on/off or adjust PWM

  for(int i = 0; i < NUM_LEDS; i++) {
    if(ledState[i]==STATE_ON){
      //      digitalWrite(ledPins[i], HIGH);
      if (analogState[i] != brightness) analogWrite(ledPins[i], brightness); // don't analogWrite unnecessarily!
      analogState[i] = brightness;
    }
    else if (ledState[i]==STATE_OFF){
      digitalWrite(ledPins[i], LOW);
      analogState[i] = 0;
    }
    else if (ledState[i]==STATE_BLINK && blinkState==1){
      //      digitalWrite(ledPins[i], HIGH);
      if (analogState[i] != brightness) analogWrite(ledPins[i], brightness); // don't analogWrite unnecessarily!
      analogState[i] = brightness;
    }
    else if (ledState[i]==STATE_BLINK && blinkState==0){
      digitalWrite(ledPins[i], LOW);
      analogState[i] = 0;
    }
    else if (ledState[i]==STATE_BLINKFAST && fastBlinkState==1){
      //      digitalWrite(ledPins[i], HIGH);
      if (analogState[i] != brightness) analogWrite(ledPins[i], brightness); // don't analogWrite unnecessarily!
      analogState[i] = brightness;
    }
    else if (ledState[i]==STATE_BLINKFAST && fastBlinkState==0){
      digitalWrite(ledPins[i], LOW);
      analogState[i] = 0;
    }
  }

} // END doLeds()

void getVolts(){
  voltsAdc = analogRead(VOLTPIN);
  voltsAdcAvg = average(voltsAdc, voltsAdcAvg);
  volts = adc2volts(voltsAdcAvg);

  minusAdc = analogRead(MINUS_VOLTPIN);
  minusAvg = average(minusAdc, minusAvg);
  minusRail = adc2volts(minusAvg); // measuring the minus rail

  plusRail = volts - minusRail; // calculating the plus rail

  //  brightness = 255 - BRIGHTNESSBASE * (1.0 - (brightnessKnobFactor * (1023 - analogRead(knobpin))));  // the knob affects brightnes

  brightness = BRIGHTNESSBASE;  // full brightness unless dimming is required
  if (volts > BRIGHTNESSVOLTAGE)
    brightness -= (BRIGHTNESSFACTOR * (volts - BRIGHTNESSVOLTAGE));  // brightness is reduced by overvoltage
  // this means if voltage is 28 volts over, PWM will be 255 - (28*4.57) or 127, 50% duty cycle
}

float average(float val, float avg){
  if(avg == 0)
    avg = val;
  return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
}

float adc2volts(float adc){
  return adc * (1 / VOLTCOEFF);
}

void printDisplay(){
  Serial.print("plusRail: ");
  Serial.print(plusRail);
  Serial.print("v   minusRail: ");
  Serial.print(minusRail);
  Serial.print("v   totalVolts: ");
  Serial.print(volts);
  Serial.print("v (");
  Serial.print(analogRead(VOLTPIN));
  Serial.print(") ");
  if (decision == PLUSPEDAL ) Serial.println("PLUSPEDAL ");
  if (decision == MINUSPEDAL) Serial.println("MINUSPEDAL");
  if (decision == OPENPEDAL ) Serial.println("OPENPEDAL ");
  //  Serial.print(", voltsBuck: ");
  //  Serial.print(voltsBuck);
  //  Serial.print(", inverter: ");
  //  Serial.print(volts-voltsBuck);
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
    case 1:
      mode = 0x01;
      break;
    case 8:
      mode = 0x02;
      break;
    case 64:
      mode = 0x03;
      break;
    case 256:
      mode = 0x04;
      break;
    case 1024:
      mode = 0x05;
      break;
    default:
      return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    }
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if(pin == 3 || pin == 11) {
    switch(divisor) {
    case 1:
      mode = 0x01;
      break;
    case 8:
      mode = 0x02;
      break;
    case 32:
      mode = 0x03;
      break;
    case 64:
      mode = 0x04;
      break;
    case 128:
      mode = 0x05;
      break;
    case 256:
      mode = 0x06;
      break;
    case 1024:
      mode = 0x7;
      break;
    default:
      return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
