#define BAUD_RATE 57600
#define DEBUG 1 // set to 1 to enable serial information printing
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
 * 2.1 - JS => changed to split_rail_48v_4level, adding PWM for LED pedalometer, turning off buck converter and sign output
 * 2.15 - JS => fixed so white LEDs are solid before starting to blink at 50v, tuned relay voltages
 * 2.2 - JS => create branch 1b1i for onebike-oneinverter which buck converts up to 60V down to 12V for inverter
 * 2.3 - JS => create branch decida for split-rail system with automatic rail selection for pedallers (see decida.xcf)
 * 2.4 - JS => rip out a bunch of stuff that we haven't used in a long time
 * 2.5 - JS => create branch sledge for ten-line sLEDgehammer pedalpower lightshow reactor
*/
char versionStr[] = "Split-Rail 48 volt 4-line pedalometer Pedal Power Utility Box ver. 2.5 branch:sledge";

// PINS
#define RELAYPIN 2 // relay cutoff output pin // NEVER USE 13 FOR A RELAY
#define VOLTPIN A0 // Voltage Sensor Pin
#define AMPSPIN A3 // Current Sensor Pin
#define NUM_LEDS 10 // Number of LED outputs.
const int ledPins[NUM_LEDS] = {
  3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

// levels at which each LED turns on (not including special states)
const float ledLevels[NUM_LEDS+1] = {
  14, 16, 18, 19, 20, 21, 22, 22.5, 23.25, 24, 0 }; // last value unused in sledge
//  24.0, 32.0, 40.0, 48.0, 50.0};

#define BRIGHTNESSVOLTAGE 24.0  // voltage at which LED brightness starts to fold back
#define BRIGHTNESSBASE 255  // maximum brightness value (255 is max value here)
int brightness = 0;  // analogWrite brightness value, updated by getVoltageAndBrightness()
#define BRIGHTNESSFACTOR (BRIGHTNESSBASE / BRIGHTNESSVOLTAGE) / 2 // results in half PWM at double voltage
// for every volt over BRIGHTNESSVOLTAGE, pwm is reduced by BRIGHTNESSFACTOR from BRIGHTNESSBASE

// FAKE AC POWER VARIABLES
#define KNOBPIN A4
int knobAdc = 0;
void doKnob(){ // look in calcWatts() to see if this is commented out
  knobAdc = 1013 - analogRead(KNOBPIN); // clockwise 50K knob wired on two-conductor cable to 50K resistor
  if (knobAdc < 0) knobAdc = 0; // values 0-10 count as zero
  knobAdc *= 2; // 50K knob wired on two-conductor cable to 50K resistor
}

int analogState[NUM_LEDS] = {0}; // stores the last analogWrite() value for each LED
                                 // so we don't analogWrite unnecessarily!

#define AVG_CYCLES 50 // average measured values over this many samples
#define DISPLAY_INTERVAL 500 // when auto-display is on, display every this many milli-seconds
#define LED_UPDATE_INTERVAL 1000
#define D4_AVG_PERIOD 10000
#define BLINK_PERIOD 600
#define FAST_BLINK_PERIOD 150

#define STATE_OFF 0
#define STATE_BLINK 1
#define STATE_BLINKFAST 3
#define STATE_ON 2

// on/off/blink/fastblink state of each led
int ledState[NUM_LEDS] = {
  STATE_OFF};

#define MAX_VOLTS 25.5  //
#define RECOVERY_VOLTS 24.0
int relayState = STATE_OFF;

#define DANGER_VOLTS 26.0
int dangerState = STATE_OFF;

int blinkState = 0;
int fastBlinkState = 0;

#define VOLTCOEFF 13.179  // larger number interprets as lower voltage

int voltsAdc = 0;
float voltsAdcAvg = 0;
float volts,realVolts = 0;

#define IDLING 0 // haven't been pedaled yet, or after draining is over
#define CHARGING 1 // someone is pedalling, at least not letting voltage fall
#define FAILING 2 // voltage has fallen in the past 30 seconds, so we drain
#define VICTORY 3 // the winning display is activated until we're drained
#define PLAYING 4 // the winning display is activated until we're drained
#define JUSTBEGAN 5


int situation = IDLING; // what is the system doing?

#define WINTIME 3000 // how many milliseconds you need to be at top level before you win
#define LOSESECONDS 30 // how many seconds ago your voltage is compared to see if you gave up
#define VRSIZE 40 // must be greater than LOSESECONDS but not big enough to use up too much RAM

float voltRecord[VRSIZE] = { 0 }; // we store voltage here once per second
int vRIndex = 0; // keep track of where we store voltage next
unsigned long vRTime = 0; // last time we stored a voltRecord

//Current related variables
int ampsAdc = 0;
float ampsAdcAvg = 0;
float amps = 0;
float volts2SecondsAgo = 0;

float watts = 0;
float wattHours = 0;
float voltsBefore = 0;
// timing variables for various processes: led updates, print, blink, etc
unsigned long time = 0;
unsigned long timeFastBlink = 0;
unsigned long timeBlink = 0;
unsigned long timeDisplay = 0;
unsigned long wattHourTimer = 0;
unsigned long victoryTime = 0; // how long it's been since we declared victory
unsigned long topLevelTime = 0; // how long we've been at top voltage level
unsigned long timefailurestarted = 0;
unsigned long timeArbduinoTurnedOn = 0;
unsigned long clearlyLosingTime = 0; // time when we last were NOT clearly losing
unsigned long serialTime = 0; // time when last serial data was seen
unsigned long drainedTime = 0; // time when volts was last OVER 13.5v
#define EMPTYTIME 1000 // how long caps must be below 13.5v to be considered empty
#define SERIALTIMEOUT 500 // if serial data is older than this, ignore it
#define SERIALINTERVAL 300 // how much time between sending a serial packet
byte presentLevel = 0;  // what "level" of transistors are we lit up to right now?

float voltishFactor = 1.0; // multiplier of voltage for competitive purposes
float voltish = 0; // this is where we store the adjusted voltage

int timeSinceVoltageBeganFalling = 0;
// var for looping through arrays
int i = 0;
int boxNumber; // short pin A5 to ground on boxNumber 1

void setup() {
  Serial.begin(BAUD_RATE);

  if (DEBUG) Serial.println(versionStr);

  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN,LOW);

  boxNumber = 1;  // box 1 if pin A5 is shorted to ground
  digitalWrite(A5,HIGH);  // enable pull-up resistor on A5
  if (digitalRead(A5)) boxNumber = 2; // if A5 is not shorted to ground

  // init LED pins
  for(i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i],OUTPUT);
      digitalWrite(ledPins[i],LOW);
  }
  situation = JUSTBEGAN;
  timeDisplay = millis();
  timeArbduinoTurnedOn = timeDisplay;
  vRTime = timeDisplay; // initialize vRTime since it's a once-per-second thing
}

void loop() {
  time = millis();
  getVolts();
  doSafety();
  realVolts = volts; // save realVolts for printDisplay function
  fakeVoltage(); // adjust 'volts' according to knob

/*
Situation is either:
IDLING
PLAYING
FAILING
(LEVEL 10)
VICTORY DANCE
DANCE OVER
HARD RESET

IDLING only goes to PLAYING if volts increases by a real amount indicating pedaling

PLAYING goes to FAILING if voltage keeps falling for at least 30 seconds

PLAYING goes to VICTORY DANCE if volts >= win voltage with all lights on for 3 seconds

FAILING always goes to IDLING before PLAYING. Lift relay after 15 seconds!

VICTORY goes to FAILING after light sequence

You know you're IDLING if volts < 14 and haven't risen and not PLAYING

You know you're PLAYING if volts have risen significantly.

You know you're FAILING if someone set it to FAILING

Ways to improve:
When someone gives up and the knob is easy, the volts falls slowly. then the game never resets. we need 'time since pedaling.'

2014-4-28 emailed pseudocode

Both sLEDgehammers need their own difficulty knob to cover the possibility of doing two events in two places on the same day.

The two boxes also need to be able to 'talk' to each other through a serial cable that I am hoping you can make.

IF the boxes are connected to each other through this cable, follow this pseudocode:

0. Give the boxes names like Box1 and Box2

1. Create a new state called "CLEARLYWINNING". Clearly winning means that you are at least 2 stages ahead of your opponent for more than 2 seconds. So if you are on
level 8 and they are on level 6 or below for more than 2 seconds, you are CLEARLYWINNING.

If Box1 is CLEARLYWINNING, adjust the 'voltish' of Box2, making it easier for them to catch up. This makes the game closer. Start with a 10% adjustment. Reset the
CLEARLYWINNING clock. If CLEARLYWINNING again, do another 10% adjustment.

2. If Box1 = VICTORY, then Box1 enters its victory dance which you will see in the code. Box2 should do a 'failure droop' where it fades out. Note the function
turnThemOffOneAtATime(). This may be useful if for nothing other than to see the timing that I like ( delay of 200ms between levels).  NOTE that the top level STAYS
ON. This is to prevent people from slipping off the pedals when the Halogens turn off and they are still pedaling hard.

3. If Box2 loses a competition because Box1 gets to victory first, then make Box2's state = FAILING. As you'll see in the code, the only way you can get out of
FAILING is for voltage (actual, not adjusted) to fall below 13.5 .


*/

  if (time - vRTime > 1000) { // we do this once per second exactly
    if(situation == JUSTBEGAN){
       if (time-timeArbduinoTurnedOn > 2200) situation = IDLING;
    }
    if ( voltish < volts2SecondsAgo + 0.1) { // stuck or slow drift
        timeSinceVoltageBeganFalling++;
      } else {
        timeSinceVoltageBeganFalling = 0;
      }

    vRTime += 1000; // add a second to the timer index
    voltRecord[vRIndex] = voltish; // store the value. JAKE doing vRIndex++ didn't work. needed to be on two separate lines.
    vRIndex++;
    if (vRIndex >= VRSIZE) vRIndex = 0; // wrap the counter if necessary
    // What's the situation?

  }
  // Am I idling?

  if (volts < 12 && situation != PLAYING && situation != JUSTBEGAN) {
    situation = IDLING;
  }

  volts2SecondsAgo =  voltRecord[(vRIndex + VRSIZE - 2) % VRSIZE]; // voltage LOSESECONDS ago

  if (situation==IDLING){

    if (voltish - volts2SecondsAgo > 0.4){ // need to get past startup sequences

      situation = PLAYING;
      timeSinceVoltageBeganFalling = 0;
      voltsBefore = voltish;
      resetVoltRecord();
      if (DEBUG) Serial.println("got to PLAYING 1");// pedaling has begun in earnest

    }

  }

  if (timeSinceVoltageBeganFalling > 15 && volts>13.5 && situation != FAILING){
    Serial.println("Got to Failing. Voltage has been falling for 15 seconds. ");
    situation=FAILING;
  }

  if (situation != VICTORY && situation == PLAYING) { // if we're not in VICTORY mode...
    voltsBefore =  voltRecord[(vRIndex + VRSIZE - LOSESECONDS) % VRSIZE]; // voltage LOSESECONDS ago
    if (timeSinceVoltageBeganFalling > 15) {
      if (DEBUG) Serial.println("Got to Failing. Voltage has been falling for 15 seconds. ");
      situation=FAILING;
    } else if ((voltsBefore - voltish) > 3) { // if voltage has fallen but they haven't given up
      if (DEBUG) Serial.print("voltsBefore: ");
      if (DEBUG) Serial.println(voltsBefore);
      situation = FAILING; // forget it, you lose
      if (DEBUG) Serial.println("got to FAILING 2");
      timefailurestarted = time;
    }
  }

  if (presentLevel < 9) { // voltish < ledLevels[NUM_LEDS-1]){
      topLevelTime = time; // reset timer unless you're at top level
  }

  if ((situation == PLAYING) && (time - topLevelTime > WINTIME) && (presentLevel == 9)) { // it's been WINTIME milliseconds of solid top-level action!

    if (situation != VICTORY) {
      victoryTime = time; // record the start time of victory
    }
    situation = VICTORY;
    if (DEBUG) Serial.print("got to VICTORY 1");
  }

  //  getAmps();  // only if we have a current sensor
  //  calcWatts(); // also adds in knob value for extra wattage, unless commented out

  //  if it's been at least 1/4 second since the last time we measured Watt Hours...
  doBlink();  // blink the LEDs
  doLeds();

  if(time - timeDisplay > DISPLAY_INTERVAL){
    printDisplay();
    timeDisplay = time;
  }

}

#define FAKEDIVISOR 2900 // 2026 allows doubling of voltage, 3039 allows 50% increase, etc..
float fakeVoltage() {
  doKnob(); // read knob value into knobAdc
  float multiplier = (float)FAKEDIVISOR / (float)(FAKEDIVISOR - knobAdc);
  volts = volts * multiplier; // turning knob up returns higher voltage

  // JAKE -- research how to do 'return'. It wasn't working so I changed to the volts = ... above.

} // if knob is all the way down, voltage is returned unchanged

void  resetVoltRecord() {
  for(i = 0; i < VRSIZE; i++) {
    voltRecord[i] = volts;
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

/*
1. advance from voltage level 1 to 10, each time turning on more stripes of
light. Here are the voltage targets: 14,16,18, 19, 20, 21, 22, 22.5, 23.25, 24
2. If the person gets to level 10 and holds it for more than 3 seconds, do
the reward sequence.
3. Reward: bottom to top with 0.1S intervals 3 times, then ALL ON to get rid
of the power and bring it back down so it's ready for the next pedaler.
4. We will use a difficulty knob. The Voltish concept worked well in the
past. All the way left means a 30% reduction in voltage targets. I need to
know how to wire it.
5. We will possibly need to implement some type of averaging or hysteresis
so it someone is out of the saddle pedaling, the lights don't pulse to their
pedaling.
* if the voltage doesn't increase for 30 seconds, turn on the halogens to
  drain the patient to 12v
*/

void doLeds(){

  presentLevel = 0; // we will now load presentLevel with highest level achieved
  for(i = 0; i < NUM_LEDS; i++) {
    if(voltish >= ledLevels[i]){
      ledState[i]=STATE_ON;
      presentLevel = i; // presentLevel should equal the highest LED level
    }
    else
      ledState[i]=STATE_OFF;
  }

  if (situation == VICTORY) presentLevel = 10; // tell the other box we won!
  if (situation == FAILING) presentLevel = 0; // tell the other box the sad truth

  if (dangerState){
    for(i = 0; i < NUM_LEDS; i++) {
      ledState[i] = STATE_ON; // try to keep the voltage down
    }
  }

  if (situation == VICTORY) { // assuming victory is not over
    if (time - victoryTime <= 3000){
      for (i = 0; i < NUM_LEDS - 1; i++) {
        ledState[i]=STATE_OFF; // turn them all off but the top one, which helps keep it from suddenly feeling easy.
      }
      ledState[((time - victoryTime) % 1000) / 100]=STATE_ON; // turn on one at a time, bottom to top, 0.1 seconds each
    } else { // 1st victory sequence is over
      situation=FAILING;
      if (DEBUG) Serial.println("I switched to FAILING 1");
      timefailurestarted = time;
    }
  }

  if (situation == FAILING){
      for (i = 0; i < NUM_LEDS; i++) {
        if (i > 6) {  // WHICH LEVELS ARE ON DURING FAILING / DRAINING
          ledState[i]=STATE_ON;
        } else {
          ledState[i]=STATE_OFF;
        }
      }
  }

  if (situation == IDLING){
    for (i = 0; i < NUM_LEDS; i++) {
      // WHICH LEVELS ARE ON DURING FAILING / DRAINING
      ledState[i]=STATE_OFF;
    }
  }

  // loop through each led and turn on/off or adjust PWM

  for(i = 0; i < NUM_LEDS; i++) {
    if(ledState[i]==STATE_ON){
      digitalWrite(ledPins[i], HIGH);
      //      if (analogState[i] != brightness) analogWrite(ledPins[i], brightness); // don't analogWrite unnecessarily!
      analogState[i] = brightness;
    }
    else if (ledState[i]==STATE_OFF){
      digitalWrite(ledPins[i], LOW);
      analogState[i] = 0;
    }
    else if (ledState[i]==STATE_BLINK && blinkState==1){
      digitalWrite(ledPins[i], HIGH);
      //      if (analogState[i] != brightness) analogWrite(ledPins[i], brightness); // don't analogWrite unnecessarily!
      analogState[i] = brightness;
    }
    else if (ledState[i]==STATE_BLINK && blinkState==0){
      digitalWrite(ledPins[i], LOW);
      analogState[i] = 0;
    }
    else if (ledState[i]==STATE_BLINKFAST && fastBlinkState==1){
      digitalWrite(ledPins[i], HIGH);
      //      if (analogState[i] != brightness) analogWrite(ledPins[i], brightness); // don't analogWrite unnecessarily!
      analogState[i] = brightness;
    }
    else if (ledState[i]==STATE_BLINKFAST && fastBlinkState==0){
      digitalWrite(ledPins[i], LOW);
      analogState[i] = 0;
    }
  }

} // END doLeds()

void doSafety() {
  if (volts > MAX_VOLTS){
    digitalWrite(RELAYPIN, HIGH);
    relayState = STATE_ON;
    if (DEBUG) Serial.println("RELAY OPEN");
  }

  if (relayState == STATE_ON && situation != FAILING && volts < RECOVERY_VOLTS){
    digitalWrite(RELAYPIN, LOW);
    relayState = STATE_OFF;
    if (DEBUG) Serial.println("RELAY CLOSED");
  }

  if (volts > DANGER_VOLTS){
    dangerState = STATE_ON;
  } else {
    dangerState = STATE_OFF;
  }

  if (situation == FAILING && relayState!=STATE_ON && (time - timefailurestarted) > 10000 ) {
    // Open the Relay so volts can drop;
    digitalWrite(RELAYPIN, HIGH);
    relayState = STATE_ON;
    if (DEBUG) Serial.println("FAILING 10seconds: RELAY OPEN");
  }
  if (volts > 13.5) {
    drainedTime = time;
  }
  if ((time - drainedTime > EMPTYTIME) && situation == FAILING ){
    situation = IDLING; //FAILING worked! we brought the voltage back to under 14.
    delay(2000);
    timeSinceVoltageBeganFalling = 0;
    digitalWrite(RELAYPIN, LOW);
    relayState = STATE_OFF;
    if (DEBUG) Serial.println("EMPTYTIME, got to IDLING 1: RELAY CLOSED");
  }
}



void getAmps(){
  ampsAdc = analogRead(AMPSPIN);
  ampsAdcAvg = average(ampsAdc, ampsAdcAvg);
  amps = adc2amps(ampsAdcAvg);
}

void getVolts(){
  voltsAdc = analogRead(VOLTPIN);
  voltsAdcAvg = average(voltsAdc, voltsAdcAvg);
  volts = adc2volts(voltsAdcAvg);

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

float adc2amps(float adc){
  return (adc - 512) * 0.1220703125;
}

void calcWatts(){
  watts = volts * amps;
//  doKnob(); // only if we have a knob to look at
//  watts += knobAdc / 2; // uncomment this line too
}

void calcWattHours(){
  wattHours += (watts * ((time - wattHourTimer) / 1000.0) / 3600.0); // measure actual watt-hours
  //wattHours +=  watts *     actual timeslice / in seconds / seconds per hour
  // In the main loop, calcWattHours is being told to run every second.
}

void printWatts(){
  if (DEBUG) Serial.print("w");
  if (DEBUG) Serial.println(watts);
}

void printWattHours(){
  if (DEBUG) Serial.print("w"); // tell the sign to print the following number
  //  the sign will ignore printed decimal point and digits after it!
  if (DEBUG) Serial.println(wattHours,1); // print just the number of watt-hours
  //  if (DEBUG) Serial.println(wattHours*10,1); // for this you must put a decimal point onto the sign!
}

void printDisplay(){
  if (DEBUG) Serial.print(realVolts);
  if (DEBUG) Serial.print("v ");
  if (DEBUG) Serial.print(volts);
  if (DEBUG) Serial.print("fv ");
  if (DEBUG) Serial.print(knobAdc);
  if (DEBUG) Serial.print("knobAdc ");
  if (DEBUG && voltishFactor > 1.0) Serial.print(voltish);
  if (DEBUG && voltishFactor > 1.0) Serial.print("voltish ");
  // if (DEBUG) Serial.print(analogRead(VOLTPIN));
  if (DEBUG) Serial.print("   Situation: ");
  if (DEBUG) Serial.print(situation);
  if (DEBUG) Serial.print("  time - topLevelTime: ");
  if (DEBUG) Serial.print(time - topLevelTime);
  if (DEBUG) Serial.print("  Voltage has been flat or falling for ");
  if (DEBUG) Serial.print(timeSinceVoltageBeganFalling);
  if (DEBUG) Serial.print(" seconds. & volts2Secondsago = ");
  if (DEBUG) Serial.println(volts2SecondsAgo);

  //   if (DEBUG) Serial.print("   ledLevels[numLEDS]: ");
 // if (DEBUG) Serial.println(ledLevels[NUM_LEDS]);
   //    if (DEBUG) Serial.print("   ledLevels[numLEDS- 1]: ");
 // if (DEBUG) Serial.println(ledLevels[NUM_LEDS - 1]); JAKE
  //  if (DEBUG) Serial.print(", a: ");
  //  if (DEBUG) Serial.print(amps);
  //  if (DEBUG) Serial.print(", va: ");
  //  if (DEBUG) Serial.print(watts);

  //  if (DEBUG) Serial.print(", Levels ");
  //  for(i = 0; i < NUM_LEDS; i++) {
  //    if (DEBUG) Serial.print(i);
  //    if (DEBUG) Serial.print(": ");
  //    if (DEBUG) Serial.print(ledState[i]);
  //    if (DEBUG) Serial.print(", ");
  //  }
  //  if (DEBUG) Serial.println("");
  // if (DEBUG) Serial.println();
}
