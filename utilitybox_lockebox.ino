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
 * 2.8 - MPS => create branch lockebox for dual utility box for Locke Elementary with Turtle Vision
*/
char versionStr[] = "Single-Rail 24 volt dual utility box ver. 2.8 branch:lockebox";

#define BOX_ID 0

// PINS
// NEVER USE 13 FOR A RELAY:
// Some bootloaders flash pin 13; that could arc a relay or damage equipment
// see http://arduino.cc/en/Hacking/Bootloader
#define RELAYPIN 2 // relay cutoff output pin
#define NUM_TEAMS 2


#define VOLTPIN A0 // Voltage Sensor Pin
const int PIN_FOR_BIKE_CURRENT[NUM_TEAMS] = { A3, A2 };  // Current Sensor Pins for bikes
const int PIN_FOR_INVERTER_CURRENT = A5;  // Current Sensor Pins for inverter

#define AVG_CYCLES 50 // average measured values over this many samples
#define LONG_AVG_CYCLES 10000  // "long" average (eg team effort) measured values over this many samples (running a bit above 1kHz)
#define DISPLAY_INTERVAL 500 // when auto-display is on, display every this many milli-seconds
#define UPDATE_INTERVAL 500 // update watcher every this many milli-seconds
#define BLINK_PERIOD 600
#define FAST_BLINK_PERIOD 150

#define STATE_OFF 0
#define STATE_BLINK 1
#define STATE_BLINKFAST 3
#define STATE_ON 2

#define MAX_VOLTS 27.0
#define RECOVERY_VOLTS 26.0
int relayState = STATE_OFF;

#define DANGER_VOLTS 27.4
int dangerState = STATE_OFF;

#define VOLTCOEFF 13.179  // larger number interprets as lower voltage

int voltsAdc = 0;
float voltsAdcAvg = 0;
float volts = 0;

// Current-related variables (in Amps unless raw ADC reading)
int ampsAdc = 0;
const float reading_for_bike_at_0A[NUM_TEAMS] = { 508, 510 };  // TODO:  calibrate
const float reading_for_bike_at_3A[NUM_TEAMS] = { 481, 483 };  // TODO:  calibrate
const float amp_scale_for_bike[NUM_TEAMS] = {
  3 / ( reading_for_bike_at_3A[0] - reading_for_bike_at_0A[0] ),
  3 / ( reading_for_bike_at_3A[1] - reading_for_bike_at_0A[1] ) };
const float reading_for_inverter_at_0A = 508;  // TODO:  calibrate
const float reading_for_inverter_at_3A = 481;  // TODO:  calibrate
const float amp_scale_for_inverter = 3 / ( reading_for_inverter_at_3A - reading_for_inverter_at_0A );
float current_for_team[NUM_TEAMS];
float current_for_inverter;

// (Instantaneous) Power-related variables (in Watts)
float power_for_team[NUM_TEAMS];
float power_out;

// (Accumulated) Energy-related variables (in Joules)
float energy_for_team[NUM_TEAMS] = { 0, 0 };
float energy_out = 0;

// timing variables for various processes: print, blink, etc
unsigned long time = 0;
unsigned long prev_time = 0;  // time of previous loop and its measurement
unsigned long loopcount = 0;
unsigned long timeDisplay = 0;
unsigned long timeUpdate = 0;

// var for looping through arrays
int i = 0;

void setup() {
  Serial.begin(BAUD_RATE);

  if (DEBUG) Serial.println(versionStr);

  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN,LOW);

  timeDisplay = millis();
}

void loop() {
  time = millis();
  if( ! prev_time )  prev_time = time;  // special case for first loop
  getVolts();
  doSafety();
  updateTeamEfforts();
  updateInverterUsage();

  if(time - timeDisplay > DISPLAY_INTERVAL){
    if(DEBUG) printDisplay();
    timeDisplay = time;
  }
  if(time - timeUpdate > UPDATE_INTERVAL){
    updateWatcher();
    timeUpdate = time;
  }

  loopcount++;
  prev_time = time;
}


void doSafety() {
  if (relayState == STATE_OFF && volts > MAX_VOLTS) {
    digitalWrite(RELAYPIN, HIGH);
    relayState = STATE_ON;
    if (DEBUG) Serial.println("RELAY OPEN");
    // we have no LEDS and no halogens, so can't do much to keep the voltage down
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


// keep a decaying average of each team's amperage
void updateTeamEfforts() {
  for( i=0; i<NUM_TEAMS; i++ ) {
    ampsAdc = ( analogRead(PIN_FOR_BIKE_CURRENT[i]) - reading_for_bike_at_0A[i] ) * amp_scale_for_bike[i];
    current_for_team[i] = long_average(ampsAdc, current_for_team[i]);
    power_for_team[i] = current_for_team[i] * volts;
    energy_for_team[i] += power_for_team[i] * (time-prev_time)/1000;
  }
}

void updateInverterUsage() {
  ampsAdc = ( analogRead(PIN_FOR_INVERTER_CURRENT) - reading_for_inverter_at_0A ) * amp_scale_for_inverter;
  current_for_inverter = long_average(ampsAdc, current_for_inverter);
  power_out = current_for_inverter * volts;
  energy_out += power_out * (time-prev_time)/1000;
}

void getVolts(){
  voltsAdc = analogRead(VOLTPIN);
  voltsAdcAvg = average(voltsAdc, voltsAdcAvg);
  volts = adc2volts(voltsAdcAvg);
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


void printDisplay(){
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
  Serial.print(volts);
  Serial.print("v ");
  Serial.print("   relayState: ");
  Serial.print(relayState);
  Serial.print("  efforts:");
#ifdef DISPLAY_RAW_CURRENTS
  Serial.print("[");
  Serial.print( analogRead(PIN_FOR_BIKE_CURRENT[0]) );
  Serial.print("]");
#endif
  Serial.print(current_for_team[0]);
  Serial.print( ',' );
  Serial.print(current_for_team[1]);
#ifdef DISPLAY_RAW_CURRENTS
  Serial.print("[");
  Serial.print( analogRead(PIN_FOR_BIKE_CURRENT[1]) );
  Serial.print("]");
#endif
  Serial.println();
}

void updateWatcher() {
#if NUM_TEAMS != 2
#error This function assumes we have exactly two teams
#endif
  Serial.print( "{\"BOX_ID\":" );
  Serial.print( BOX_ID );
  Serial.print( ",\"VOLTS\":" );
  Serial.print( volts );
  Serial.print( ",\"BIKES\":[" );
  for( i=0; i<NUM_TEAMS; i++ ) {
    Serial.print( "{\"AMPS\":" );
    Serial.print( current_for_team[i] );
    Serial.print( ",\"WATTS\":" );
    Serial.print( power_for_team[i] );
    Serial.print( ",\"ENERGY\":" );
    Serial.print( energy_for_team[i] );
    Serial.print( "}" );
    if( i != NUM_TEAMS-1 )
      Serial.print( "," );
  }
  Serial.print( "],\"WATTS_IN\":" );
  Serial.print( power_for_team[0] + power_for_team[1] );
  Serial.print( ",\"WATTS_OUT\":" );
  Serial.print( power_out );
  Serial.print( ",\"ENERGY_IN\":" );  // was TOTAL_IN
  // we store consistently in Watt-sec; have to display in Watt-hr
  Serial.print( ( energy_for_team[0] + energy_for_team[1] ) / 60 );
  Serial.print( ",\"ENERGY_OUT\":" );  // was TOTAL_OUT
  // we store consistently in Watt-sec; have to display in Watt-hr
  Serial.print( energy_out / 60 );
  Serial.print( ",\"SAMPLES\":" );
  Serial.print( loopcount );
  Serial.print( ",\"REL_TIME\":" );
  Serial.print( time - timeUpdate );
  Serial.print( ",\"ARDUINO_TIME\":" );
  Serial.print( time );
  Serial.println( "}" );
}
