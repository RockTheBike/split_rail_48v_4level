#define BAUD_RATE 57600
/**** Single-rail Pedalometer
 * Arduino code to run the Dance with Lance Arbduino
 * ver. 1.14
 * Written by:
 * Thomas Spellman <thomas@thosmos.com>
 * Jake <jake@spaz.org>
 * Paul@rockthebike.com
 *
 * Notes:
 * 1.6 - moved version to the top, started protocol of commenting every change in file and in Git commit
 * 1.7 - jake 6-21-2012 disable minusalert until minus rail is pedaled at least once (minusAlertEnable and startupMinusVoltage)
 * 1.8 -- FF added annoying light sequence for when relay fails or customer bypasses protection circuitry.+
 * 1.9 - TS => cleaned up a bit, added state constants, turn off lowest 2 levels when level 3 and above
 * 1.10 - TS => cleaned up levels / pins variables, changed to a "LEDs" naming scheme
 * 1.11 - TS => does a very slow 4digits watts average, fixed the high blink
 * 1.12 - TS => printWatts uses D4Avg instead of watts, 300 baud
 * 1.13 - TS => D4Avg fix, 2400 baud
 * 1.14 - FF => Added CalcWattHours function, changing the Sign's data to Watt Hours, instead of Watts, in time for BMF VII
 * 1.15 - JS => started adding buck converter stuff 
 * 2.09 - JS => changed to split_rail_48v_4level, adding PWM for LED pedalometer  */
char versionStr[] = "AC Power Pedal Power Utility Box ver. 1.14. For best results connect the Sign!";

#define brightnessVoltage  28.0  // voltage at which LED brightness starts to fold back
#define brightnessBase 255  // maximum brightness value (255 is max value here)
int brightness = 0;  // analogWrite brightness value, updated by getVoltageAndBrightness()
#define brightnessFactor 4.57 // the factor by which PWM is reduced upon higher system voltage for LEDs

// FAKE AC POWER VARIABLES
const int knobPin = A2;
int knobAdc = 0;
void doKnob(){
  knobAdc = analogRead(knobPin) - 10; // make sure not to add if knob is off
  if (knobAdc < 0) knobAdc = 0; // values 0-10 count as zero
}

// GLOBAL VARIABLES
const int AVG_CYCLES = 50; // average measured values over this many samples
const int DISPLAY_INTERVAL = 2000; // when auto-display is on, display every this many milli-seconds
const int LED_UPDATE_INTERVAL = 1000;
const int D4_AVG_PERIOD = 10000;
const int BLINK_PERIOD = 600;
const int FAST_BLINK_PERIOD = 150;

// STATE CONSTANTS
const int STATE_OFF = 0;
const int STATE_BLINK = 1;
const int STATE_BLINKFAST = 3;
const int STATE_ON = 2;

// LEDS
const int NUM_LEDS = 4; // Number of LED outputs.
// levels at which each LED turns on (not including special states)
float ledLevels[NUM_LEDS] = {
  24.0, 32.0, 40.0, 48.0};
//  24.0, 28.0, 32.0, 36.0, 40.0, 44.0, 48.0};
// current active level
int ledLevel = -1;
// on/off/blink/fastblink state of each led
int ledState[NUM_LEDS] = {
  STATE_OFF};

// PINS
const int relayPin = 2; // relay cutoff output pin // NEVER USE 13 FOR A RELAY
const int voltPin = A0; // Voltage Sensor Pin
const int ampsPin = A3; // Current Sensor Pin
const int ledPins[NUM_LEDS] = {
  //  2, 3, 4, 5, 6, 7, 8};
  3, 9, 10, 11};

// SPECIAL STATE
const float MAX_VOLTS = 50.0;  //
const float RECOVERY_VOLTS = 40.0;
int relayState = STATE_OFF;

const float DANGER_VOLTS = 52.0;
int dangerState = STATE_OFF;

int blinkState = 0;
int fastBlinkState = 0;

const float voltcoeff = 13.25;  // larger number interprets as lower voltage

//Voltage related variables.
int voltsAdc = 0;
float voltsAdcAvg = 0;
float volts = 0;

int voltsBuckAdc = 0; // for measuring A1 voltage
float voltsBuckAvg = 0; // for measuring A1 voltage
float voltsBuck = 0; // averaged A1 voltage

//Current related variables
int ampsAdc = 0;
float ampsAdcAvg = 0;
float amps = 0;

float watts = 0;
float wattHours = 0;

int readCount = 0; // for determining how many sample cycle occur per display interval
int avgCount = 0;
volatile float D4Avg = 0.0;
float D4AvgCycles = 0;
boolean D4Initted = false;

// timing variables for various processes: led updates, print, blink, etc
unsigned long time = 0;
unsigned long timeFastBlink = 0;
unsigned long timeBlink = 0;
unsigned long timeRead = 0;
unsigned long timeDisplay = 0;
unsigned long timeLeds = 0;
unsigned long wattHourTimer = 0;


// var for looping through arrays
int i = 0;
int x = 0;
int y = 0;

void setup() {
  Serial.begin(BAUD_RATE);

  Serial.println(versionStr);

  pinMode(voltPin,INPUT);
  pinMode(ampsPin,INPUT);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin,LOW);

  // init LED pins
  for(i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i],OUTPUT);
    digitalWrite(ledPins[i],LOW);
  }

  timeDisplay = millis();
  setPwmFrequency(3,1); // this sets the frequency of PWM on pins 3 and 11 to 31,250 Hz
  setPwmFrequency(9,1); // this sets the frequency of PWM on pins 9 and 10 to 31,250 Hz
  //  pinMode(9,OUTPUT); // this pin will control the transistors of the huge BUCK converter
}

//int senseLevel = -1;

void loop() {
  time = millis();
  getVolts();
  //  doBuck(); // adjust inverter voltage
  doSafety();
  //  getAmps();
  readCount++;
  //  calcWatts();

  //  if it's been at least 1/4 second since the last time we measured Watt Hours...
  /*  if (time - wattHourTimer >= 250) {
   calcWattHours();
   wattHourTimer = time; // reset the integrator    
   }
   
   if(avgCount > AVG_CYCLES && D4Initted){
   //tmpD4Avg = D4average(watts, D4Avg);
   D4average();
   //    Serial.print("recalc watts: ");
   //    Serial.print(watts);
   //    Serial.print(", tmpD4Avg: ");
   //    Serial.println(tmpD4Avg);
   //    Serial.print("recalcing D4Avg: ");
   //    Serial.println(D4Avg);
   avgCount = 0;
   //D4Avg = tmpD4Avg;
   }
   */
  //Now show the - Team how hard to pedal.
  if(time - timeDisplay > DISPLAY_INTERVAL){
    /*
    // set up the 4D avg cycles
     if(!D4Initted){
     D4AvgCycles = (30.0 * (float)readCount) / (float)AVG_CYCLES;
     //      Serial.print("readCount: ");
     //      Serial.println(readCount);
     //      Serial.print("D4AvgCycles: ");
     //      Serial.println(D4AvgCycles);
     D4Initted = true;
     }
     */

    doBlink();  // blink the LEDs

    doLeds();

    // printWatts();
    //    printWattHours();
    printDisplay();
    //readCount = 0;
    timeDisplay = time;
  }

}

float p1, p2, p3 = 0.0;

float D4average(){
  if(D4Avg < 0.01)
    D4Avg = 0.01;
  //  Serial.print("D4average: watts: ");
  //  Serial.print(watts);
  //  Serial.print(", D4Avg: ");
  //  Serial.print(D4Avg);
  //  Serial.print(", D4AvgCycles: ");
  //  Serial.print(D4AvgCycles);
  //  if(avg == 0.0)
  //    avg = val;
  p1 = D4Avg * (D4AvgCycles - 1);
  //  Serial.print(", p1: ");
  //  Serial.println(p1);
  p2 = watts + p1;
  //  Serial.print(", p2: ");
  //  Serial.println(p2);
  p3 = p2 / D4AvgCycles;
  //  Serial.print(", p3: ");
  //  Serial.println(p3);
  //return p3;
  D4Avg = p3;
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

void doSafety() {
  if (volts > MAX_VOLTS){
    digitalWrite(relayPin, HIGH);
    relayState = STATE_ON;
  }

  if (relayState == STATE_ON && volts < RECOVERY_VOLTS){
    digitalWrite(relayPin, LOW);
    relayState = STATE_OFF;
  }

  if (volts > DANGER_VOLTS){
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

  // Set the desired lighting states.

  ledLevel = -1;

  for(i = 0; i < NUM_LEDS; i++) {
    if(volts >= ledLevels[i]){
      ledLevel = i;
      ledState[i]=STATE_ON;
    }
    else
      ledState[i]=STATE_OFF;
  }

  // if voltage is below the lowest level, blink the lowest level
  if (volts < ledLevels[0]){
    ledLevel=0;
    ledState[0]=STATE_BLINK;
  }

  // turn off first 2 levels if voltage is above 3rd level
  if(volts > ledLevels[1]){
    ledState[0] = STATE_OFF;
//    ledState[1] = STATE_OFF;
  }

  if (dangerState){
    for(i = 0; i < NUM_LEDS; i++) {
      ledState[i] = STATE_BLINKFAST;
    }
  }

  // if at the top level, blink it fast
  if (ledLevel == (NUM_LEDS-1)){
    ledState[ledLevel] = STATE_BLINKFAST;
  }

  // Do the desired states.
  // loop through each led and turn on/off or adjust PWM

  for(i = 0; i < NUM_LEDS; i++) {
    if(ledState[i]==STATE_ON){
      //      digitalWrite(ledPins[i], HIGH);
      analogWrite(ledPins[i], brightness);
    }
    else if (ledState[i]==STATE_OFF){
      digitalWrite(ledPins[i], LOW);
    }
    else if (ledState[i]==STATE_BLINK && blinkState==1){
      //      digitalWrite(ledPins[i], HIGH);
      analogWrite(ledPins[i], brightness);
    }
    else if (ledState[i]==STATE_BLINK && blinkState==0){
      digitalWrite(ledPins[i], LOW);
    }
    else if (ledState[i]==STATE_BLINKFAST && fastBlinkState==1){
      //      digitalWrite(ledPins[i], HIGH);
      analogWrite(ledPins[i], brightness);
    }
    else if (ledState[i]==STATE_BLINKFAST && fastBlinkState==0){
      digitalWrite(ledPins[i], LOW);
    }
  }

} // END doLeds()


int ampsCompensation = 2; // wtf is this?
void getAmps(){
  //  ampsAdc = analogRead(ampsPin);
  //  ampsAdc = analogRead(ampsPin);
  ampsAdc = analogRead(ampsPin);
  ampsAdc += ampsCompensation;
  ampsAdcAvg = average(ampsAdc, ampsAdcAvg);
  amps = adc2amps(ampsAdcAvg);
  avgCount++;
}


void getVolts(){
  voltsAdc = analogRead(voltPin);
  voltsAdcAvg = average(voltsAdc, voltsAdcAvg);
  volts = adc2volts(voltsAdcAvg);

  voltsBuckAdc = analogRead(BUCK_VOLTPIN);
  voltsBuckAvg = average(voltsBuckAdc, voltsBuckAvg);
  voltsBuck = adc2volts(voltsBuckAvg);

  //  brightness = 255 - brightnessBase * (1.0 - (brightnessKnobFactor * (1023 - analogRead(knobpin))));  // the knob affects brightnes

  brightness = brightnessBase;  // full brightness unless dimming is required
  if (volts > brightnessVoltage)
    brightness -= (brightnessFactor * (volts - brightnessVoltage));  // brightness is reduced by overvoltage
  // this means if voltage is 28 volts over, PWM will be 255 - (28*4.57) or 127, 50% duty cycle
}

float average(float val, float avg){
  if(avg == 0)
    avg = val;
  return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
}

static int volts2adc(float v){

  //adc = v * 10/110/5 * 1024 == v * 18.618181818181818;

  return v * voltcoeff;
}



float adc2volts(float adc){
  // v = adc * 110/10 * 5 / 1024 == adc * 0.0537109375;
  return adc * (1 / voltcoeff); // 55 / 1024 = 0.0537109375;
}

// amp sensor conversion factors
// 0A == 512 adc == 1.65pV // current sensor offset
// pV/A = .04 pV/A (@5V) * 3.3V/5V = .0264 pV/A (@3.3V) // sensor sensitivity (pV = adc input pin volts)
// adc/pV = 1024 adc / 3.3 pV = 310.3030303030303 adc/pV  // adc per pinVolt
// adc/A = 310.3030303030303 adc/pV * 0.0264 pV/A = 8.192 adc/A
// A/adc = 1 A / 8.192 adc = 0.1220703125 A/adc

float adc2amps(float adc){
  // A/adc = 0.1220703125 A/adc
  return (adc - 512) * 0.1220703125;
  //return adc * (1 / ampcoeff);
}

void calcWatts(){
  watts = volts * amps;
  doKnob();
  watts += knobAdc / 2;
  //Serial.print("calcWatts: ");
  //Serial.println(watts);
}

void calcWattHours(){
  wattHours += (watts * ((time - wattHourTimer) / 1000.0) / 3600.0); // measure actual watt-hours
  //wattHours +=  watts *     actual timeslice / in seconds / seconds per hour

  /* This code was written to show accumulated Watt Hours at events. 
   The 0.0278 factor is 100 divided by the number of seconds in an hour.
   In the main loop you can see that calcWattHours is being told to run every second.
   The number printed to the sign is actual watt hours * 10. 
   So if it says 58, you can tell the pedaler, "you just pedaled 5.8 WattHours. Thanks!" 
   Before BMF, change the factor to 0.00278. (why?)
   Then the number printed on the Sign will be actual Watt Hours.   */
}

void printWatts(){
  Serial.print("w");
  Serial.println(D4Avg);
}

void printWattHours(){
  Serial.print("w"); // tell the sign to print the following number
  //  the sign will ignore printed decimal point and digits after it!
  Serial.println(wattHours,1); // print just the number of watt-hours
  //  Serial.println(wattHours*10,1); // for this you must put a decimal point onto the sign!
}

void printDisplay(){
  Serial.print(volts);
  Serial.print("v (");
  Serial.println(analogRead(voltPin));
  //  Serial.print(", a: ");
  //  Serial.print(amps);
  //  Serial.print(", va: ");
  //  Serial.print(watts);
  //  Serial.print(", voltsBuck: ");
  //  Serial.print(voltsBuck);
  //  Serial.print(", inverter: ");
  //  Serial.print(volts-voltsBuck);

  //  Serial.print(", Levels ");
  //  for(i = 0; i < NUM_LEDS; i++) {
  //    Serial.print(i);
  //    Serial.print(": ");
  //    Serial.print(ledState[i]);
  //    Serial.print(", ");
  //  }
  //  Serial.println("");
  Serial.println();

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













