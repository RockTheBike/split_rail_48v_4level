#define BAUD_RATE 57600
#define DEBUG 1 // set to 1 to enable serial information printing
/**** Pedalometer / sLEDgehammer
 * Written by:
 * Thomas Spellman <thomas@thosmos.com>
 * Jake <jake@spaz.org>
 * Paul@rockthebike.com
 * mark@rockthebike.com
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
 * 2.6 - MPS => create branch solarliving for sLEDgehammer for Solar Living Center
 * 2.7 - MPS => create branch dual for sLEDgehammer for Solar Living Center
*/
char versionStr[] = "Single-Rail 24 volt dualing sLEDgehammer ver. 2.7 branch:dual";

// PINS
// NEVER USE 13 FOR A RELAY:
// Some bootloaders flash pin 13; that could arc a relay or damage equipment
// see http://arduino.cc/en/Hacking/Bootloader
#define RELAYPIN 2 // relay cutoff output pin
#define NUM_TEAMS 2
#define NUM_COLUMNS 5

// indexing into ledPins
#define LED_FOR_SINK 10  // ie halogen energy sink
const int LED_FOR_TEAM_COLUMN[NUM_TEAMS][NUM_COLUMNS] = {
  { 0, 1, 2, 3, 4 },
  { 5, 6, 7, 8, 9 } };

#define VOLTPIN A0 // Voltage Sensor Pin
const int AMPSPINS[NUM_TEAMS] = { A3, A2 };  // Current Sensor Pins
#define NUM_LEDS 12 // Number of LED outputs (includes (halogen) energy sinks)
const int ledPins[NUM_LEDS] = {  3,4,5,6,7, 8,  9,10,11,12,A5, 13  };

#define BRIGHTNESSVOLTAGE 24.0  // voltage at which LED brightness starts to fold back
#define BRIGHTNESSBASE 255  // maximum brightness value (255 is max value here)
int brightness = 0;  // analogWrite brightness value, updated by getVoltageAndBrightness()
#define BRIGHTNESSFACTOR (BRIGHTNESSBASE / BRIGHTNESSVOLTAGE) / 2 // results in half PWM at double voltage
// for every volt over BRIGHTNESSVOLTAGE, pwm is reduced by BRIGHTNESSFACTOR from BRIGHTNESSBASE

int analogState[NUM_LEDS] = {0}; // stores the last analogWrite() value for each LED
                                 // so we don't analogWrite unnecessarily!

#define AVG_CYCLES 50 // average measured values over this many samples
#define LONG_AVG_CYCLES 5000  // "long" average (eg team effort) measured values over this many samples
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

#define MAX_VOLTS 27.0
#define RECOVERY_VOLTS 26.0
int relayState = STATE_OFF;

#define DANGER_VOLTS 27.4
int dangerState = STATE_OFF;

int blinkState = 0;
int fastBlinkState = 0;

#define VOLTCOEFF 13.179  // larger number interprets as lower voltage

int voltsAdc = 0;
float voltsAdcAvg = 0;
float volts,realVolts = 0;

# define THERMOMETER_STATE 0
# define PARTY_STATE 1
# define DRAIN_STATE 2
int gameState = THERMOMETER_STATE;

#define VRSIZE 40 // must be greater than LOSESECONDS but not big enough to use up too much RAM

float voltRecord[VRSIZE] = { 0 }; // we store voltage here once per second
int vRIndex = 0; // keep track of where we store voltage next
unsigned long vRTime = 0; // last time we stored a voltRecord

//Current related variables
int ampsAdc = 0;
float ampsAdcAvg[NUM_TEAMS];
const float ampsBase[NUM_TEAMS] = { 511.00, 507.85 };  // measurement with zero current
const float ampsScale[NUM_TEAMS] = { 1, -1 };
float amps = 0;
int winning_team;
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
    digitalWrite(ledPins[i],LOW);   // does this work with A5?
  }
  timeDisplay = millis();
  timeArbduinoTurnedOn = timeDisplay;
  vRTime = timeDisplay; // initialize vRTime since it's a once-per-second thing
}

void loop() {
  time = millis();
  getVolts();
  doSafety();
  updateTeamEfforts();
  realVolts = volts; // save realVolts for printDisplay function

  if (time - vRTime > 1000)  // we do this once per second exactly
    updateVoltRecord();

  if(!dangerState) {
    playGame();
  }

  doBlink();  // blink the LEDs
  doLeds();

  if(time - timeDisplay > DISPLAY_INTERVAL){
    printDisplay();
    timeDisplay = time;
  }

}

void updateVoltRecord() {
  if ( voltish < volts2SecondsAgo + 0.1) { // stuck or slow drift
    timeSinceVoltageBeganFalling++;
  } else {
    timeSinceVoltageBeganFalling = 0;
  }
  vRTime += 1000; // add a second to the timer index
  voltRecord[vRIndex] = voltish; // store the value. JAKE doing vRIndex++ didn't work. needed to be on two separate lines.
  vRIndex++;
  if (vRIndex >= VRSIZE) vRIndex = 0; // wrap the counter if necessary
}

void  resetVoltRecord() {
  for(i = 0; i < VRSIZE; i++) {
    voltRecord[i] = volts;
  }
}


void playGame() {
    gameState = partyAnimation();
    return;




  switch(gameState) {
  case THERMOMETER_STATE:
    gameState = thermometerAnimation();
  break;
  case PARTY_STATE:
    gameState = partyAnimation();
  break;
  case DRAIN_STATE:
    gameState = drainAnimation();
  break;
  }
}

int thermometerAnimation() {
  #define VICTORY_THRESHOLD 12.5
  // we control the column LEDs with some combo of voltage and accumulated team effort
  static const float threshold_for_column_led[] = { 6.0, 8.0, 9.25, 10.5, 11.5 };
  for( int team=0; team<NUM_TEAMS; team++ )
    for( int col=0; col<NUM_COLUMNS; col++ ) {
      float creditedVolts = voltish * ampsAdcAvg[team] / max(ampsAdcAvg[0],ampsAdcAvg[1]);
      ledState[LED_FOR_TEAM_COLUMN[team][col]] =
        creditedVolts > threshold_for_column_led[col] ? STATE_ON : STATE_OFF;
    }
  ledState[LED_FOR_SINK] = STATE_OFF;
  // TODO:  if( no_one's_given_energy_in_5s ) return DRAIN_STATE;
  return voltish > VICTORY_THRESHOLD ? PARTY_STATE : THERMOMETER_STATE;
}

int partyAnimation() {
  #define SUSTAINED_VICTORY_THRESHOLD 11.0
  static int millis_until_next_frame = 2000;
  static int old_frame_index;
  static int new_frame_index = 0;
  static unsigned long time_for_next_frame;
  // turn on halogen sink so sudden drop in load doesn't overpower capacitor
  const int frames_single_clockwise[][NUM_LEDS] = {
    { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
    { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
    { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1 },
    { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 },
    { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1 },
    { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1 },
    { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
    { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
  const int frames_double_wide[][NUM_LEDS] = {
    { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
    { 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1 },
    { 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1 },
    { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1 },
    { 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1 },
    { 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1 },
    { 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1 },
    { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1 },
    { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
  const int frames_opposite[][NUM_LEDS] = {
    { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 },
    { 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1 },
    { 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1 },
    { 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1 } };
  #define frames frames_double_wide
  // advance to (or initialize) the next frame when necessary
  if( time >= time_for_next_frame ) {
    // we want 0->2000, 14->50, 13->100
    // m = a * v**2 + b * v + c
    // 2000 = a * 0**2 + b * 0 + c
    // 50 = a * 14**2 + b * 14 + c
    // 100 = a * 13**2 + b * 13 + c
    // c = 2000
    // -1950 = a * 14**2 + b * 14
    // -1900 = a * 13**2 + b * 13
    // -1950 = a * 196 + b * 14
    // -1900 = a * 169 + b * 13
    // -25350 = a * 2548 + b * 182
    // -26600 = a * 2366 + b * 182
    // 1250 = a * 182
    // 6.868 = a
    // -1900 = 6.868 * 169 + b * 13
    // -235.4378 = b
    millis_until_next_frame = 6.868 * volts*volts + -235.4378 * volts + 2000;
    millis_until_next_frame = 200;
    time_for_next_frame =
      ( time_for_next_frame ? time_for_next_frame : time ) + millis_until_next_frame;
    old_frame_index = new_frame_index;
    new_frame_index = (new_frame_index+1) % (sizeof(frames)/sizeof(*frames));
  }
  // PWM via picking between frames with increasing probability of later frame
  int frame_index = rand() < RAND_MAX / millis_until_next_frame * ( time_for_next_frame - time ) ? old_frame_index : new_frame_index;
  for( i=0; i<NUM_LEDS; i++ )
    ledState[i] = frames[frame_index][i] ? STATE_ON : STATE_OFF;
  return voltish > SUSTAINED_VICTORY_THRESHOLD ? PARTY_STATE : DRAIN_STATE;
}

int drainAnimation() {
  #define DRAINED_THRESHOLD 6.0
  for( i=0; i<NUM_LEDS; i++ )
    ledState[i] = STATE_ON;
  return voltish < DRAINED_THRESHOLD ? THERMOMETER_STATE : DRAIN_STATE;
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
  if (relayState == STATE_OFF && volts > MAX_VOLTS) {
    digitalWrite(RELAYPIN, HIGH);
    relayState = STATE_ON;
    if (DEBUG) Serial.println("RELAY OPEN");
    // try to keep the voltage down
    for(i = 0; i < NUM_LEDS; i++) {
      digitalWrite(ledPins[i], HIGH);
    }
    delay(2000);
  }

  if (relayState == STATE_ON && volts < RECOVERY_VOLTS){
    digitalWrite(RELAYPIN, LOW);
    relayState = STATE_OFF;
    if (DEBUG) Serial.println("RELAY CLOSED");
  }

  if (volts > DANGER_VOLTS){
    dangerState = STATE_ON;
  } else {
    dangerState = STATE_OFF;
  }
}


// keep a decaying average of each team's aperage;
// use the ratio to determine who's winning
void updateTeamEfforts() {
  for( i=0; i<NUM_TEAMS; i++ ) {
    ampsAdc = ( analogRead(AMPSPINS[i]) - ampsBase[i] ) * ampsScale[i];
    ampsAdcAvg[i] = long_average(ampsAdc, ampsAdcAvg[i]);
  }
  winning_team = ampsAdcAvg[0] > ampsAdcAvg[1];
}

void getVolts(){
  voltsAdc = analogRead(VOLTPIN);
  voltsAdcAvg = average(voltsAdc, voltsAdcAvg);
  volts = adc2volts(voltsAdcAvg);

  brightness = BRIGHTNESSBASE;  // full brightness unless dimming is required
  if (volts > BRIGHTNESSVOLTAGE)
    brightness -= (BRIGHTNESSFACTOR * (volts - BRIGHTNESSVOLTAGE));  // brightness is reduced by overvoltage
  // this means if voltage is 28 volts over, PWM will be 255 - (28*4.57) or 127, 50% duty cycle

  voltish = volts;
}

float average(float val, float avg){
  if(avg == 0)
    avg = val;
  return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
}

float long_average(float val, float avg){
  if(avg == 0)
    avg = val;
  return (val + (avg * (LONG_AVG_CYCLES - 1))) / LONG_AVG_CYCLES;
}

float adc2volts(float adc){
  return adc * (1 / VOLTCOEFF);
}

float adc2amps(float adc){
  return (adc - 512) * 0.1220703125;
}

void calcWatts(){
  watts = volts * amps;
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
  if (DEBUG && voltishFactor > 1.0) Serial.print(voltish);
  if (DEBUG && voltishFactor > 1.0) Serial.print("voltish ");
  // if (DEBUG) Serial.print(analogRead(VOLTPIN));
  if (DEBUG) Serial.print("   relayState: ");
  if (DEBUG) Serial.print(relayState);
  if (DEBUG) Serial.print("  gameState: ");
  if (DEBUG) Serial.print(gameState);
  if (DEBUG) Serial.print("  efforts:");
  if (DEBUG) Serial.print(ampsAdcAvg[0]);
  if (DEBUG) Serial.print( winning_team ? '<' : '>' );
  if (DEBUG) Serial.print(ampsAdcAvg[1]);
  if (DEBUG) Serial.println();
}
