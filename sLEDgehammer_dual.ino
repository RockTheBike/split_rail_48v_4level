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
const int LED_FOR_TEAM_SINKS[NUM_TEAMS] = { 5, 11 };  // ie halogen energy sinks
const int LED_FOR_TEAM_COLUMN[NUM_TEAMS][NUM_COLUMNS] = {
  { 0, 1, 2, 3, 4 },
  { 6, 7, 8, 9, 10 } };

#define VOLTPIN A0 // Voltage Sensor Pin
const int AMPSPINS[NUM_TEAMS] = { A3, A2 };  // Current Sensor Pins
#define NUM_LEDS 12 // Number of LED outputs (includes (halogen) energy sinks)
const int ledPins[NUM_LEDS] = {  3,4,5,6,7, 8,  9,10,11,12,A5, 13  };

#define AVG_CYCLES 50 // average measured values over this many samples
#define LONG_AVG_CYCLES 10000  // "long" average (eg team effort) measured values over this many samples (running a bit above 1kHz)
#define DISPLAY_INTERVAL 500 // when auto-display is on, display every this many milli-seconds
#define BLINK_PERIOD 600
#define FAST_BLINK_PERIOD 150

#define STATE_OFF 0
#define STATE_BLINK 1
#define STATE_BLINKFAST 3
#define STATE_ON 2

// on/off/blink/fastblink state of each led
int ledState[NUM_LEDS] = {
  STATE_OFF};

//#define DISABLE_THERMOMETER_PWM
#ifdef DISABLE_ALL_PWM
#define DISABLE_THERMOMETER_PWM
#define DISABLE_PARTY_PWM
#define DISABLE_DRAIN_PWM
#endif

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
int gameState = DRAIN_STATE;

//Current related variables
int ampsAdc = 0;
float ampsAdcAvg[NUM_TEAMS] = { 0, 0 };
const float ampsBase[NUM_TEAMS] = { 508, 510 };  // measurement with zero current
const float rawAmpsReadingAt3A[NUM_TEAMS] = { 481, 483 };
const float ampsScale[NUM_TEAMS] = {
  3 / ( rawAmpsReadingAt3A[0] - ampsBase[0] ),
  3 / ( rawAmpsReadingAt3A[1] - ampsBase[1] ) };
int winning_team;

// timing variables for various processes: led updates, print, blink, etc
unsigned long time = 0;
#define DISPLAY_TIME
#ifdef DISPLAY_TIME
unsigned long loopcount = 0;
#endif
unsigned long timeFastBlink = 0;
unsigned long timeBlink = 0;
unsigned long timeDisplay = 0;

float voltishFactor = 1.0; // multiplier of voltage for competitive purposes
float voltish = 0; // this is where we store the adjusted voltage

int timeSinceVoltageBeganFalling = 0;
// var for looping through arrays
int i = 0;

void setup() {
  Serial.begin(BAUD_RATE);

  if (DEBUG) Serial.println(versionStr);

  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN,LOW);

  // init LED pins
  for(i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i],OUTPUT);
    digitalWrite(ledPins[i],LOW);
  }
  timeDisplay = millis();
}

void loop() {
  time = millis();
  static unsigned long next_volts_reading = 0;
  if( time > next_volts_reading ) {
    getVolts();
    next_volts_reading = time + 500;
  }
  doSafety();
  updateTeamEfforts();
  realVolts = volts; // save realVolts for printDisplay function

  if(!dangerState) {
    playGame();
  }

  doBlink();  // blink the LEDs
  doLeds();

  if(time - timeDisplay > DISPLAY_INTERVAL){
    if(DEBUG) printDisplay();
    timeDisplay = time;
  }

#ifdef DISPLAY_TIME
  loopcount++;
#endif
}


void playGame() {
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


#define VICTORY_THRESHOLD 22.0
// a column should start to glow at its threshold_for_column_led
// and reach PWM 100% at its higher neighbor's threshold_for_column_led
static const float threshold_for_column_led[] = { 12.5, 14.5, 16.5, 18.5, 20.5, VICTORY_THRESHOLD };
int thermometerAnimation() {
  // we control the column LEDs with some combo of voltage and accumulated team effort
  for( int team=0; team<NUM_TEAMS; team++ ) {
    float creditedVolts = creditVolts( team );
    for( int col=0; col<NUM_COLUMNS; col++ )
      ledState[LED_FOR_TEAM_COLUMN[team][col]] =
        creditedVolts < threshold_for_column_led[col] ? STATE_OFF :
#ifdef DISABLE_THERMOMETER_PWM
        STATE_ON;
#else
        creditedVolts > threshold_for_column_led[col+1] ? STATE_ON :
        rand() < RAND_MAX *
          (                   creditedVolts - threshold_for_column_led[col] ) /
          ( threshold_for_column_led[col+1] - threshold_for_column_led[col] ) ?
          STATE_ON : STATE_OFF;
#endif
    ledState[LED_FOR_TEAM_SINKS[team]] = STATE_OFF;
  }
  // TODO:  if( no_one's_given_energy_in_5s ) return DRAIN_STATE;
  if( voltish > VICTORY_THRESHOLD ) {
    enterPartyState();
    return PARTY_STATE;
  }
  return THERMOMETER_STATE;
}


// state for party mode
unsigned long victory_time;
int won_team;
float losingCreditedVolts;

void enterPartyState() {
  victory_time = time;
  won_team = winning_team;
  losingCreditedVolts = creditVolts( ! won_team );
}

int partyAnimation() {
  #define SUSTAINED_VICTORY_THRESHOLD 21.0
  #define FLUFFING_THRESHOLD 26
  partyAnimationWinner();
  partyAnimationLoser();
  // add load to make winner work to sustain party mode
  ledState[LED_FOR_TEAM_SINKS[won_team]] = STATE_ON;
  // turn on loser's halogen if we fear voltage that would trip relay
  ledState[LED_FOR_TEAM_SINKS[!won_team]] = volts > FLUFFING_THRESHOLD ? STATE_ON : STATE_OFF;
  return voltish > SUSTAINED_VICTORY_THRESHOLD ? PARTY_STATE : DRAIN_STATE;
}

int partyAnimationWinner() {
  static int millis_until_next_frame;
  static int old_frame_index;
  static int new_frame_index = 0;
  static unsigned long time_for_next_frame = 0;
  // turn on at least one halogen sink so sudden drop in load doesn't overpower capacitor
  const int frames[][NUM_COLUMNS] = {
    { 1, 0, 0, 0, 0 },
    { 1, 1, 0, 0, 0 },
    { 0, 1, 1, 0, 0 },
    { 0, 0, 1, 1, 0 },
    { 0, 0, 0, 1, 1 },
    { 0, 0, 0, 0, 1 },
    { 0, 0, 0, 0, 0 } };
  // advance to (or initialize) the next frame when necessary
  if( time >= time_for_next_frame ) {
    const int fast_interval=25;  // at MAX_VOLTS
    const int slow_interval=100;  // at SUSTAINED_VICTORY_THRESHOLD
    millis_until_next_frame =  // linear interpolation
      ( fast_interval *
          (     volts - SUSTAINED_VICTORY_THRESHOLD ) +
        slow_interval *
          ( MAX_VOLTS - volts ) ) /
      (     MAX_VOLTS - SUSTAINED_VICTORY_THRESHOLD     );
    time_for_next_frame =
      ( time_for_next_frame ? time_for_next_frame : time ) + millis_until_next_frame;
    old_frame_index = new_frame_index;
    new_frame_index = (new_frame_index+1) % (sizeof(frames)/sizeof(*frames));
  }
#ifdef DISABLE_PARTY_PWM
  int frame_index = old_frame_index;
#else
  // PWM via picking between frames with increasing probability of later frame
  int frame_index = rand() < RAND_MAX / millis_until_next_frame * ( time_for_next_frame - time ) ? old_frame_index : new_frame_index;
#endif
  for( i=0; i<NUM_COLUMNS; i++ )
    ledState[LED_FOR_TEAM_COLUMN[won_team][i]] =
      frames[frame_index][i] ? STATE_ON : STATE_OFF;
}

#define SLUICE_TIME 3000
int partyAnimationLoser() {
  float creditedVolts = time > victory_time + SLUICE_TIME ?
    0 : losingCreditedVolts * (victory_time+SLUICE_TIME-time) / SLUICE_TIME;
  for( int col=0; col<NUM_COLUMNS; col++ )
    ledState[LED_FOR_TEAM_COLUMN[!won_team][col]] =
      creditedVolts > threshold_for_column_led[col] ? STATE_ON : STATE_OFF;
}


int drainAnimation() {
  #define DRAINED_THRESHOLD 12.0
  static int millis_until_next_frame;
  static int old_frame_index;
  static int new_frame_index = 0;
  static unsigned long time_for_next_frame = 0;
  // turn on halogen sinks so sudden drop in load doesn't overpower capacitor
  const int frames[][NUM_LEDS] = {
    {  1,1,1,1,1, 1,  1,1,1,1,1, 1  },
    {  1,1,1,1,0, 1,  1,1,1,1,0, 1  },
    {  1,1,1,0,1, 1,  1,1,1,0,1, 1  },
    {  1,1,0,1,1, 1,  1,1,0,1,1, 1  },
    {  1,0,1,1,1, 1,  1,0,1,1,1, 1  },
    {  0,1,1,1,1, 1,  0,1,1,1,1, 1  } };
  // advance to (or initialize) the next frame when necessary
  if( time >= time_for_next_frame ) {
    // interactive animation would only encourage people to pedal
    millis_until_next_frame = 200;
    time_for_next_frame =
      ( time_for_next_frame ? time_for_next_frame : time ) + millis_until_next_frame;
    old_frame_index = new_frame_index;
    new_frame_index = (new_frame_index+1) % (sizeof(frames)/sizeof(*frames));
  }
#ifdef DISABLE_DRAIN_PWM
  int frame_index = old_frame_index;
#else
  // PWM via picking between frames with increasing probability of later frame
  int frame_index = rand() < RAND_MAX / millis_until_next_frame * ( time_for_next_frame - time ) ? old_frame_index : new_frame_index;
#endif
  for( i=0; i<NUM_LEDS; i++ )
    ledState[i] = frames[frame_index][i] ? STATE_ON : STATE_OFF;
  if( voltish < DRAINED_THRESHOLD ) {
    ampsAdcAvg[0] = ampsAdcAvg[1] = 0;  // forget recent efforts
    return THERMOMETER_STATE;
  }
  return DRAIN_STATE;
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
    }
    else if (ledState[i]==STATE_OFF){
      digitalWrite(ledPins[i], LOW);
    }
    else if (ledState[i]==STATE_BLINK && blinkState==1){
      digitalWrite(ledPins[i], HIGH);
    }
    else if (ledState[i]==STATE_BLINK && blinkState==0){
      digitalWrite(ledPins[i], LOW);
    }
    else if (ledState[i]==STATE_BLINKFAST && fastBlinkState==1){
      digitalWrite(ledPins[i], HIGH);
    }
    else if (ledState[i]==STATE_BLINKFAST && fastBlinkState==0){
      digitalWrite(ledPins[i], LOW);
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
  winning_team = ampsAdcAvg[0] < ampsAdcAvg[1];
}

void getVolts(){
  voltsAdc = analogRead(VOLTPIN);
  voltsAdcAvg = average(voltsAdc, voltsAdcAvg);
  volts = adc2volts(voltsAdcAvg);

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

// We credit each team with some fraction of the voltage in the common
// capacitor (but the fractions don't sum to 1).  The winning team gets
// credit for everything (because victory is defined to be when the winning
// team reaches the top).  The losing team gets credit proportional to the
// (recent) ratio of currents.
float creditVolts( int team ) {
  if( team==winning_team )
    return voltish;
  else if( ampsAdcAvg[team] <= 0 )
    return 0;
  else
    return voltish * ampsAdcAvg[team] / ampsAdcAvg[winning_team];
}

void printDisplay(){
#ifdef DISPLAY_TIME
  Serial.print( loopcount );
  Serial.print( "loops " );
  Serial.print( time/1000/60 );
  Serial.print( ':' );
  Serial.print( time/1000%60 );  // TODO "%02d"
  Serial.print( ' ' );
#define DISPLAY_REFRESH_RATE
#ifdef DISPLAY_REFRESH_RATE
  Serial.print( loopcount * 1000 / time );
  Serial.print( "Hz " );
#endif
  Serial.print( ' ' );
#endif
  Serial.print(realVolts);
  Serial.print("v ");
  Serial.print(volts);
  Serial.print("fv ");
  Serial.print("   relayState: ");
  Serial.print(relayState);
  Serial.print("  gameState: ");
  Serial.print(gameState);
  Serial.print("  efforts:");
#ifdef DISPLAY_RAW_CURRENTS
  Serial.print("[");
  Serial.print( analogRead(AMPSPINS[0]) );
  Serial.print("]");
#endif
  Serial.print(ampsAdcAvg[0]);
  Serial.print( winning_team ? '<' : '>' );
  Serial.print(ampsAdcAvg[1]);
#ifdef DISPLAY_RAW_CURRENTS
  Serial.print("[");
  Serial.print( analogRead(AMPSPINS[1]) );
  Serial.print("]");
#endif
  Serial.print("   LEDs:  ");
  const char* separators[] = {
    ",", ",", ",", ",", ", ", ",  ",
    ",", ",", ",", ",", ", ", "" };
  for(i=0;i<NUM_LEDS;i++) {
    Serial.print(ledState[i]);
    Serial.print(separators[i]);
  }
  Serial.println();
}
