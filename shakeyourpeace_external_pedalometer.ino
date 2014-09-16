#define BAUD_RATE 57600
#define DEBUG 1 // set to 1 to enable serial information printing
/**** Single-rail Pedalometer
 * Arduino code to run the Dance with Lance Arbduino
 * ver. 1.14
 * Written by:
 * Thomas Spellman <thomas@thosmos.com>
 * Jake <jake@spaz.org>
 * Paul@rockthebike.com
 * Mark P Sullivan <mark@rockthebike.com>
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
 * 2.7 - MPS => create branch shakeyourpeace_external_pedalometer for Split-Rail 24 volt pedalometer with side indicator for minusrail

 * fixed > logic harder
 * turned off red LEDs when white over-voltage light turns on
*/
char versionStr[] = "Split-Rail 24 volt pedalometer with side indicator for minusrail ver. 2.7 branch:shakeyourpeace_external_pedalometer";

#define NUM_RAILS 2
#define VOLTPIN A0 // Voltage Sensor Pin
#define MINUSVOLTPIN A1 // Voltage Sensor Input for arbduino v2
const int VOLTPINS[NUM_RAILS] = { VOLTPIN, MINUSVOLTPIN };
#define NUM_LEDS 8 // Number of LED outputs
const int ledPins[NUM_LEDS] = {
  // plus pedalometer
  6,7,9,5,3,4,
  // minus degenerate-pedalometer / side-lights
  10, 11 };

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
// referencing in order:
// plus bottom through plus top, then minus bottom through minus top
#define PLUS_BOTTOM 0
#define PLUS_TOP 5
#define MINUS_BOTTOM 6
#define MINUS_TOP 7
int BOTTOM_LED[NUM_RAILS] = { PLUS_BOTTOM, MINUS_BOTTOM };
int TOP_LED[NUM_RAILS] = { PLUS_TOP, MINUS_TOP };

int blinkState = 0;
int fastBlinkState = 0;

#define VOLTCOEFF 13.179  // larger number interprets as lower voltage
const int VOLT_PRE_OFFSET[NUM_RAILS] = { 0, 1023 };
const int SCALE[NUM_RAILS] = { 1, -1 };
const float VOLT_POST_OFFSET[NUM_RAILS] = { 0, -5 };

#define NUM_LEVELS 6
const float levelVolt[NUM_LEVELS] = { 22.0, 23.5, 24.8, 25.7, 26.7, 27.2 };
int levels[NUM_RAILS];

int voltsAdc[NUM_RAILS];
float voltsAdcAvg[NUM_RAILS];
float volts[NUM_RAILS];

#define VRSIZE 40 // must be greater than LOSESECONDS but not big enough to use up too much RAM

float voltRecord[VRSIZE] = { 0 }; // we store voltage here once per second
int vRIndex = 0; // keep track of where we store voltage next
unsigned long vRTime = 0; // last time we stored a voltRecord

//Current related variables
int ampsAdc = 0;
float amps = 0;
int winning_team;
float volts2SecondsAgo = 0;

// timing variables for various processes: led updates, print, blink, etc
unsigned long time = 0;
unsigned long timeFastBlink = 0;
unsigned long timeBlink = 0;
unsigned long timeDisplay = 0;
unsigned long timeArbduinoTurnedOn = 0;

float voltishFactor = 1.0; // multiplier of voltage for competitive purposes
float voltish = 0; // this is where we store the adjusted voltage

int timeSinceVoltageBeganFalling = 0;
// var for looping through arrays
int i = 0;
int rail;

void setup() {
  Serial.begin(BAUD_RATE);

  if (DEBUG) Serial.println(versionStr);

  digitalWrite(A5,HIGH);  // enable pull-up resistor on A5

  // init LED pins
  for(i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i],OUTPUT);
      digitalWrite(ledPins[i],LOW);
  }
  timeDisplay = millis();
  timeArbduinoTurnedOn = timeDisplay;
  vRTime = timeDisplay; // initialize vRTime since it's a once-per-second thing
}

void loop() {
  time = millis();
  getVolts();

  if (time - vRTime > 1000)  // we do this once per second exactly
    updateVoltRecord();

#ifdef DEBUG_PATTERN
  debugPattern();
#else
  playGame();
#endif

  doBlink();  // blink the LEDs
  doLeds();

  if(time - timeDisplay > DISPLAY_INTERVAL){
    if(DEBUG) printDisplay();
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


void debugPattern() {
  static int cursor;
  static int onoff;
  const static int CURSOR_POSITIONS[] = { 0,0, 1, 2, 3, 4, 5, 6, 7 };
  delay(1000);
  if( onoff ) {
    onoff = 0;
    for( i=0; i<NUM_LEDS; i++ )
      ledState[i] = i==cursor ? STATE_ON : STATE_OFF;
    cursor = (cursor+1) % (sizeof(CURSOR_POSITIONS)/sizeof(*CURSOR_POSITIONS));
  } else {
    onoff = 1;
    for( i=0; i<NUM_LEDS; i++ )
      ledState[i] = STATE_OFF;
  }
}


// a visually compact table of whether each led should be on/blinking vs off
// positions:  big pedalometer: 2 red, 3 green, 1 white,  side lights: 1 red, 1 green
int LEDS_FOR_LEVEL[][NUM_LEDS] = {
  #define LEVEL_PANIC 0
  { 1,0, 0,0,0, 0,  1, 0 },
  #define LEVEL_LOW_SAFE 1
  { 1,0, 0,0,0, 0,  1, 0 },
  { 1,1, 0,0,0, 0,  0, 1 },
  { 0,0, 1,0,0, 0,  0, 1 },
  { 0,0, 1,1,0, 0,  0, 1 },
  #define LEVEL_HIGH_SAFE 4
  { 0,0, 1,1,1, 0,  0, 1 },
  #define LEVEL_OVER 5
  { 0,0, 1,1,1, 1,  1, 1 } };

void playGame() {
  for( int rail=0; rail<NUM_RAILS; rail++ ) {
    levels[rail] = ledsState( volts[rail], rail );
    for( int i=BOTTOM_LED[rail]; i<=TOP_LED[rail]; i++ )
      ledState[i] = LEDS_FOR_LEVEL[levels[rail]][i] ?
#ifdef DISABLE_BLINK
        ( volts[rail]<levelVolt[0] || volts[rail]>levelVolt[NUM_LEVELS-1] ) ?
          STATE_BLINK : STATE_ON  :
#else
        STATE_ON :
#endif
        STATE_OFF;
  }
}

int ledsState( float v, int rail ) {
  if( v < levelVolt[0] )
    // less than the lowest levelVolt => blinking LEVEL_PANIC
    return LEVEL_PANIC;
  if( v > levelVolt[NUM_LEVELS-1] )
    // more than the highest levelVolt => blinking LEVEL_OVER
    return LEVEL_OVER;
  // TODO fade in the top (or the next) segment of the mercury via PWM
  if( rail == 0 )  // plus rail
    // (first) crossing levelVolt[i] => non-blinking LEVEL_LOW_SAFE+i
    for( i=0; i<NUM_LEVELS; i++ ) {
      if( levelVolt[i] > v ) return LEVEL_LOW_SAFE+i;
      // since we got past LEVEL_OVER, we know levelVolt[NUM_LEVELS-1] >= v
    }
  else  // minus rail
    if( v >= levelVolt[2] )
      return LEVEL_LOW_SAFE+1;
    else
      return LEVEL_LOW_SAFE;
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
/*
if ((float)volts[1] < -16){
  digitalWrite(10, LOW);
digitalWrite(11, HIGH);
Serial.print("Minus Green");
Serial.println(volts[1]);
} else if ((float)volts[1] >= -16){
digitalWrite(11, HIGH);
digitalWrite(10, LOW);
Serial.print("Minus Red");
Serial.println(volts[1]);
}*/

digitalWrite(10, volts[1]<16);
digitalWrite(11, volts[1]>16);



} // END doLeds()


void getVolts(){
  for( rail=0; rail<NUM_RAILS; rail++ ) {
    voltsAdc[rail] = analogRead( VOLTPINS[rail] );
    voltsAdcAvg[rail] = average( voltsAdc[rail], voltsAdcAvg[rail] );
    volts[rail] =
      adc2volts( VOLT_PRE_OFFSET[rail] + SCALE[rail]*voltsAdcAvg[rail] ) +
      VOLT_POST_OFFSET[rail];
  }

  brightness = BRIGHTNESSBASE;  // full brightness unless dimming is required
  if (volts[0] > BRIGHTNESSVOLTAGE)
    brightness -= (BRIGHTNESSFACTOR * (volts[0] - BRIGHTNESSVOLTAGE));  // brightness is reduced by overvoltage
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
  Serial.print("+");
  Serial.print(volts[0]);
  Serial.print(",-");
  Serial.print(volts[1]);
  Serial.print("v ");
  Serial.print("   levels: ");
  Serial.print(levels[0]);
  Serial.print(",");
  Serial.print(levels[1]);
  Serial.print("   Plus rail[");
  for(i=0;i<5;i++) {
    Serial.print(ledState[i]);
    Serial.print(",");
  }
  Serial.print(ledState[5]);
  Serial.print("]  Minus rail[");
  Serial.print(ledState[6]);
  Serial.print(",");
  Serial.print(ledState[7]);
  Serial.print("]");
  Serial.println();
}
