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
 * 2.1 - JS => changed to split_rail_48v_4level, adding PWM for LED pedalometer, turning off buck converter and sign output
 * 2.15 - JS => fixed so white LEDs are solid before starting to blink at 50v, tuned relay voltages
 * 2.2 - JS => create branch 1b1i for onebike-oneinverter which buck converts up to 60V down to 12V for inverter
 * 2.3 - JS => create branch decida for split-rail system with automatic rail selection for pedallers (see decida.xcf)
 * 2.4 - JS => rip out a bunch of stuff that we haven't used in a long time
*/
char versionStr[] = "Split-Rail DIVIDA 48 volt 7-line pedalometer Pedal Power Utility Box ver. 2.4 branch buck";

// PINS
#define THERMALPIN A5 // an NTC thermistor connects from here to ground, with a pullup resistor
#define THERMAL_LIMIT 500 // BELOW this value is considered too hot

#define DIVIDAPIN 13 // transistor pulls virtual ground toward minusrail
#define DIVIDA_VOLTPIN A1 // MINUSRAIL used for a voltage divider sensor (was A2 before)
#define DIVIDA_HYSTERESIS 1.0 // how many volts above halfway before divida activated

#define RELAYPIN 2 // relay cutoff output pin // NEVER USE 13 FOR A RELAY
#define VOLTPIN A0 // Voltage Sensor Pin
#define AMPSPIN A3 // Current Sensor Pin
#define NUM_LEDS 7 // Number of LED outputs.
const int ledPins[NUM_LEDS] = {
  3, 4, 5, 6, 7, 8, 11};

// levels at which each LED turns on (not including special states)
const float ledLevels[NUM_LEDS+1] = {
  24.0, 28.0, 32.0, 36.0, 40.0, 44.0, 48.0, 50.0}; // 48.6 or 54 volts max?

#define BRIGHTNESSVOLTAGE 24.0  // voltage at which LED brightness starts to fold back
#define BRIGHTNESSBASE 255  // maximum brightness value (255 is max value here)
int brightness = 0;  // analogWrite brightness value, updated by getVoltageAndBrightness()
#define BRIGHTNESSFACTOR (BRIGHTNESSBASE / BRIGHTNESSVOLTAGE) / 2 // results in half PWM at double voltage
// for every volt over BRIGHTNESSVOLTAGE, pwm is reduced by BRIGHTNESSFACTOR from BRIGHTNESSBASE

// FAKE AC POWER VARIABLES
#define KNOBPIN A2
int knobAdc = 0;
void doKnob(){ // look in calcWatts() to see if this is commented out
  knobAdc = analogRead(KNOBPIN) - 10; // make sure not to add if knob is off
  if (knobAdc < 0) knobAdc = 0; // values 0-10 count as zero
}

int analogState[NUM_LEDS] = {0}; // stores the last analogWrite() value for each LED
                                 // so we don't analogWrite unnecessarily!

#define AVG_CYCLES 50 // average measured values over this many samples
#define DISPLAY_INTERVAL 1000 // when auto-display is on, display every this many milli-seconds
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

#define MAX_VOLTS 50.5  //
#define RECOVERY_VOLTS 44.0
int relayState = STATE_OFF;

#define DANGER_VOLTS 52.0
int dangerState = STATE_OFF;

int blinkState = 0;
int fastBlinkState = 0;

#define VOLTCOEFF 13.179  // larger number interprets as lower voltage

int thermalAdc = 0;
float thermalAdcAvg = 0;

int voltsAdc = 0;
float voltsAdcAvg = 0;
float volts = 0;

int voltsBuckAdc = 0; // for measuring A1 voltage
float voltsBuckAvg = 0; // for measuring A1 voltage
float voltsBuck = 0; // averaged A1 voltage

int voltsDividaAdc = 0; // for measuring Divida voltage
float voltsDividaAvg = 0; // for measuring Divida voltage
float voltsDivida = 0; // averaged Divida voltage

//Current related variables
int ampsAdc = 0;
float ampsAdcAvg = 0;
float amps = 0;

float watts = 0;
float wattHours = 0;

// timing variables for various processes: led updates, print, blink, etc
unsigned long time = 0;
unsigned long timeFastBlink = 0;
unsigned long timeBlink = 0;
unsigned long timeDisplay = 0;
unsigned long wattHourTimer = 0;

// var for looping through arrays
int i = 0;

void setup() {
  Serial.begin(BAUD_RATE);

  Serial.println(versionStr);

  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN,LOW);

  // init LED pins
  for(i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i],OUTPUT);
  }

  timeDisplay = millis();
  // setPwmFrequency(3,1); // this sets the frequency of PWM on pins 3 and 11 to 31,250 Hz
  setPwmFrequency(9,1); // this sets the frequency of PWM on pins 9 and 10 to 31,250 Hz
  pinMode(9,OUTPUT); // this pin will control the transistors of the huge BUCK converter
  pinMode(DIVIDAPIN,OUTPUT); // this transistor pulls virtual ground down toward minusrail
}

void loop() {
  time = millis();
  getVolts();
  doDivida(); // perform megadivida function
  doBuck(); // adjust inverter voltage
  doSafety();
  //  getAmps();  // only if we have a current sensor
  //  calcWatts(); // also adds in knob value for extra wattage, unless commented out

  //  if it's been at least 1/4 second since the last time we measured Watt Hours...
  /*  if (time - wattHourTimer >= 250) {
   calcWattHours();
   wattHourTimer = time; // reset the integrator    
   }
  */
  doBlink();  // blink the LEDs
  doLeds();

  if(time - timeDisplay > DISPLAY_INTERVAL){
    // printWatts();
    //    printWattHours();
    printDisplay();
    timeDisplay = time;
  }

}

#define BUCK_CUTIN 13 // voltage above which transistors can start working
#define BUCK_CUTOUT 11 // voltage below which transistors can not function
#define BUCK_VOLTAGE 26.0 // target voltage for inverter to be supplied with
#define BUCK_VOLTPIN A1 // this pin measures inverter's MINUS TERMINAL voltage
#define BUCK_HYSTERESIS 0.75 // volts above BUCK_VOLTAGE where we start regulatin
#define BUCK_PWM_UPJUMP 0.1 // amount to raise PWM value if voltage is below BUCK_VOLTAGE
#define BUCK_PWM_DOWNJUMP 0.5 // amount to lower PWM value if voltage is too high
float buckPWM = 0; // PWM value of pin 9
int lastBuckPWM = 0; // make sure we don't call analogWrite if already set right

void doBuck() {
  thermalAdc = analogRead(THERMALPIN); // NTC thermistor connected from ground to ADC pin, with pullup resistor
  thermalAdc = 600; // disable temp checking for now
  thermalAdcAvg = average(thermalAdc, thermalAdcAvg);
  if (thermalAdcAvg > THERMAL_LIMIT) {  // if heatsink/thermistor is not too hot (adc BELOW limit = too hot)
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
  } else digitalWrite(9,LOW); // heatsink/thermistor is too hot (adc too low)
}

void doDivida() { // perform megadivida function
  if (voltsDivida > (volts / 2 + DIVIDA_HYSTERESIS) && volts > 7) digitalWrite(DIVIDAPIN,HIGH); // pull virtual ground lower
  if (voltsDivida < (volts / 2)) digitalWrite(DIVIDAPIN,LOW); // stop pulling it down
}

void doSafety() {
  if (volts > MAX_VOLTS){
    digitalWrite(RELAYPIN, HIGH);
    relayState = STATE_ON;
  }

  if (relayState == STATE_ON && volts < RECOVERY_VOLTS){ // relay can turn off unless divida malfunctioning
    if (voltsDivida > (volts / 2 + 3 * DIVIDA_HYSTERESIS) && volts > 7) { // divida is malfunctioning
      digitalWrite(RELAYPIN, HIGH); // turn on relay because of failed divida
    } else if (voltsDivida < (volts / 2 + 2 * DIVIDA_HYSTERESIS)) { // divida is nearly fine
      digitalWrite(RELAYPIN, LOW); // turn relay off
    }
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

  for(i = 0; i < NUM_LEDS; i++) {
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

  // turn off first 2 levels if voltage is above 3rd level
  if(volts > ledLevels[2]){
    ledState[0] = STATE_OFF;
    ledState[1] = STATE_OFF;
  }

  if (dangerState){
    for(i = 0; i < NUM_LEDS; i++) {
      ledState[i] = STATE_BLINKFAST;
    }
  }

  if (volts >= ledLevels[NUM_LEDS]) {// if at the top voltage level, blink last LEDS fast
    ledState[NUM_LEDS-1] = STATE_BLINKFAST; // last set of LEDs
  }

  // loop through each led and turn on/off or adjust PWM

  for(i = 0; i < NUM_LEDS; i++) {
    if(ledState[i]==STATE_ON){
      digitalWrite(ledPins[i], HIGH);
      // if (analogState[i] != brightness) analogWrite(ledPins[i], brightness); // don't analogWrite unnecessarily!
      // analogState[i] = brightness;
    }
    else if (ledState[i]==STATE_OFF){
      digitalWrite(ledPins[i], LOW);
      analogState[i] = 0;
    }
    else if (ledState[i]==STATE_BLINK && blinkState==1){
      digitalWrite(ledPins[i], HIGH);
      // if (analogState[i] != brightness) analogWrite(ledPins[i], brightness); // don't analogWrite unnecessarily!
      // analogState[i] = brightness;
    }
    else if (ledState[i]==STATE_BLINK && blinkState==0){
      digitalWrite(ledPins[i], LOW);
      analogState[i] = 0;
    }
    else if (ledState[i]==STATE_BLINKFAST && fastBlinkState==1){
      digitalWrite(ledPins[i], HIGH);
      // if (analogState[i] != brightness) analogWrite(ledPins[i], brightness); // don't analogWrite unnecessarily!
      // analogState[i] = brightness;
    }
    else if (ledState[i]==STATE_BLINKFAST && fastBlinkState==0){
      digitalWrite(ledPins[i], LOW);
      analogState[i] = 0;
    }
  }

} // END doLeds()

void getAmps(){
  ampsAdc = analogRead(AMPSPIN);
  ampsAdcAvg = average(ampsAdc, ampsAdcAvg);
  amps = adc2amps(ampsAdcAvg);
}

void getVolts(){
  voltsAdc = analogRead(VOLTPIN);
  voltsAdcAvg = average(voltsAdc, voltsAdcAvg);
  volts = adc2volts(voltsAdcAvg);

  voltsBuckAdc = analogRead(BUCK_VOLTPIN);
  voltsBuckAvg = average(voltsBuckAdc, voltsBuckAvg);
  voltsBuck = adc2volts(voltsBuckAvg);

  voltsDividaAdc = analogRead(DIVIDA_VOLTPIN);
  voltsDividaAvg = average(voltsDividaAdc, voltsDividaAvg);
  voltsDivida = adc2volts(voltsDividaAvg);

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
  Serial.print("w");
  Serial.println(watts);
}

void printWattHours(){
  Serial.print("w"); // tell the sign to print the following number
  //  the sign will ignore printed decimal point and digits after it!
  Serial.println(wattHours,1); // print just the number of watt-hours
  //  Serial.println(wattHours*10,1); // for this you must put a decimal point onto the sign!
}

void printDisplay(){
  Serial.print(volts);
  Serial.print("v ");
  // Serial.print(analogRead(VOLTPIN));
  //  Serial.print(", a: ");
  //  Serial.print(amps);
  //  Serial.print(", va: ");
  //  Serial.print(watts);
  Serial.print(", lastBuckPWM: ");
  Serial.print(lastBuckPWM);
  //  Serial.print(", voltsBuck: ");
  //  Serial.print(voltsBuck);
  Serial.print(", inverter: ");
  Serial.print(volts-voltsBuck);
  Serial.print(", voltsDivida: ");
  if (digitalRead(DIVIDAPIN)) Serial.print("DRAINING: ");
  Serial.println(voltsDivida);
  //  Serial.print(", Levels ");
  //  for(i = 0; i < NUM_LEDS; i++) {
  //    Serial.print(i);
  //    Serial.print(": ");
  //    Serial.print(ledState[i]);
  //    Serial.print(", ");
  //  }
  //  Serial.println("");
  // Serial.println();
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
