#include <stdio.h>
#include <math.h>

int matching_level( float v, struct level levels[], int num_levels );
void playGame();
int main( int arcg, char** argv );
int eval_test_case( struct test_case* test_case );


// real code below

#define NUM_RAILS 2
#define MAX_LEDS_PER_RAIL 6
#define NUM_LEDS 8 // Number of LED outputs

#define STATE_OFF 0
#define STATE_BLINK 1
#define STATE_BLINKFAST 3
#define STATE_ON 2

// on/off/blink/fastblink state of each led
int ledState[NUM_LEDS];
// referencing in order:
// plus bottom through plus top, then minus bottom through minus top
int BOTTOM_LED[NUM_RAILS] = { 0, 6 };
int TOP_LED[NUM_RAILS] = { 5, 7 };

int levels[NUM_RAILS];
struct level {
  float threshold;
  int led_states[MAX_LEDS_PER_RAIL];
};
struct level LEVELS_FOR_PLUS_RAIL[] = {
  // threshold,  leds:red    red         green     green     green       white
  { -INFINITY, { STATE_BLINK,STATE_OFF,  STATE_OFF,STATE_OFF,STATE_OFF,  STATE_OFF } },
  { 22.0,      { STATE_ON,   STATE_OFF,  STATE_OFF,STATE_OFF,STATE_OFF,  STATE_OFF } },
  { 23.5,      { STATE_ON,   STATE_ON,   STATE_OFF,STATE_OFF,STATE_OFF,  STATE_OFF } },
  { 24.8,      { STATE_OFF,  STATE_OFF,  STATE_ON, STATE_OFF,STATE_OFF,  STATE_OFF } },
  { 25.7,      { STATE_OFF,  STATE_OFF,  STATE_ON, STATE_ON, STATE_OFF,  STATE_OFF } },
  { 26.7,      { STATE_OFF,  STATE_OFF,  STATE_ON, STATE_ON, STATE_ON,   STATE_OFF } },
  { 27.0,      { STATE_OFF,  STATE_OFF,  STATE_ON, STATE_ON, STATE_ON,   STATE_ON }  },
  { 27.2,      { STATE_OFF,  STATE_OFF,  STATE_ON, STATE_ON, STATE_ON,   STATE_BLINK } } };
struct level LEVELS_FOR_MINUS_RAIL[] = {
  // threshold,  leds:red      green
  { -INFINITY, { STATE_BLINK,  STATE_OFF   } },
  { 14.0,      { STATE_ON,     STATE_OFF   } },
  { 16.0,      { STATE_OFF,    STATE_ON    } },
  { 23.0,      { STATE_OFF,    STATE_BLINK } } };
struct level* LEVELS_FOR_RAILS[NUM_RAILS] = {
  LEVELS_FOR_PLUS_RAIL,
  LEVELS_FOR_MINUS_RAIL };
int NUM_LEVELS_FOR_RAILS[NUM_RAILS] = {
  sizeof(LEVELS_FOR_PLUS_RAIL)/sizeof(*LEVELS_FOR_PLUS_RAIL),
  sizeof(LEVELS_FOR_MINUS_RAIL)/sizeof(*LEVELS_FOR_MINUS_RAIL) };

// volts[0] holds plus rail, volts[1] holds minus rail but should be positive
float volts[NUM_RAILS];
int i = 0;

void playGame() {
  for( int rail=0; rail<NUM_RAILS; rail++ ) {
    int level = levels[rail] = matching_level(
      volts[rail], LEVELS_FOR_RAILS[rail], NUM_LEVELS_FOR_RAILS[rail] );
    for( int i=0,j=BOTTOM_LED[rail]; j<=TOP_LED[rail]; j++,i++ )
      ledState[j] = LEVELS_FOR_RAILS[rail][level].led_states[i];
  }
}

int matching_level( float v, struct level levels[], int num_levels ) {
  // TODO fade in the top (or the next) segment of the mercury via PWM
  for( i=num_levels-1; i; i-- )
    if( levels[i].threshold < v ) return i;
  return 0;
}


// real code above
// test infrastructure stuff below


struct test_case {
  float vPlus;
  float vMinus;
  int ideal_leds[NUM_LEDS];
};
struct test_case test_cases[] = {
  // raise both voltages together
  { 21.0, 13.5, { 1, 0,0,0, 0,0,  1, 0 } },
  { 23.0, 15.0, { 2, 0,0,0, 0,0,  2, 0 } },  // fails 
  { 24.5, 20.0, { 2, 2,0,0, 0,0,  0, 2 } },  // causes segv at 64 
  { 25.5, 20.0, { 0, 0,2,0, 0,0,  0, 2 } },
  { 26.5, 20.0, { 0, 0,2,2, 0,0,  0, 2 } },
  { 26.9, 20.0, { 0, 0,2,2, 2,0,  0, 2 } },
  { 27.1, 20.0, { 0, 0,2,2, 2,2,  0, 2 } },
  { 28.0, 24.0, { 0, 0,2,2, 2,1,  0, 1 } },
  // one climbs as other drops; ensure each looks at right voltage
  { 21.0, 24.0, { 1, 0,0,0, 0,0,  0, 1 } },
  { 23.0, 20.0, { 2, 0,0,0, 0,0,  0, 2 } },
  { 24.5, 20.0, { 2, 2,0,0, 0,0,  0, 2 } },
  { 25.5, 20.0, { 0, 0,2,0, 0,0,  0, 2 } },
  { 26.5, 20.0, { 0, 0,2,2, 0,0,  0, 2 } },
  { 26.9, 20.0, { 0, 0,2,2, 2,0,  0, 2 } },
  { 27.1, 15.0, { 0, 0,2,2, 2,2,  2, 0 } },
  { 28.0, 13.5, { 0, 0,2,2, 2,1,  1, 0 } },
  // some extremes
  { 10.0, 10.0, { 1, 0,0,0, 0,0,  1, 0 } },
  { 99.9, 99.9, { 0, 0,2,2, 2,1,  0, 1 } } };

int main( int arcg, char** argv ) {
  int failures = 0;
  struct test_case* test_case=test_cases;
  for( int i=0;
    i<sizeof(test_cases)/sizeof(*test_cases);
    test_case++, i++ ) {
    volts[0] = test_case->vPlus;
    volts[1] = test_case->vMinus;
    playGame();
    if( !eval_test_case(test_case) )  failures++;
  }
  if( failures ) {
    printf( "%d failures\n", failures );
    return 1;
  } else
    return 0;
}

int eval_test_case( struct test_case* test_case ) {
  int pass = 1;
  for( i=0; i<NUM_LEDS; i++ )
    if( ledState[i] != test_case->ideal_leds[i] )  pass = 0;
  printf("Actual LEDs ");
  for( i=0; i<NUM_LEDS; i++ )  printf("%d",ledState[i]);
  printf( pass ? " == " : " != " );
  printf("ideal LEDs ");
  for( i=0; i<NUM_LEDS; i++ )  printf("%d",test_case->ideal_leds[i]);
  printf( "  <==  +%2.2f,-%2.2fv\n", volts[0], volts[1] );
  return pass;
}
