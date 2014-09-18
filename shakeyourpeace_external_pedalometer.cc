#include <stdio.h>

int level( float v, struct leds_for_levels* leds_for_levels );
void playGame();
int main( int arcg, char** argv );
int eval_test_case( struct test_case* test_case );


// real code below

#define NUM_RAILS 2
#define MAX_LEVELS 8
#define MAX_LEDS_PER_RAIL 6
#define NUM_LEDS 8 // Number of LED outputs

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
struct leds_for_levels {
  const int num_thresholds;
  const float thresholds[MAX_LEVELS-1];
  // a record has one more level than it has threshold
  const int leds_for_level[MAX_LEVELS][MAX_LEDS_PER_RAIL];
} LEDS_FOR_LEVELS[NUM_RAILS] = {
  {  // plus rail
    7,
    { 22.0, 23.5, 24.8, 25.7, 26.7, 27.0, 27.2 },
    {  // positions:  2 red, 3 green, 1 white
      { STATE_BLINK,STATE_OFF,  STATE_OFF,STATE_OFF,STATE_OFF,  STATE_OFF },
      { STATE_ON,   STATE_OFF,  STATE_OFF,STATE_OFF,STATE_OFF,  STATE_OFF },
      { STATE_ON,   STATE_ON,   STATE_OFF,STATE_OFF,STATE_OFF,  STATE_OFF },
      { STATE_OFF,  STATE_OFF,  STATE_ON, STATE_OFF,STATE_OFF,  STATE_OFF },
      { STATE_OFF,  STATE_OFF,  STATE_ON, STATE_ON, STATE_OFF,  STATE_OFF },
      { STATE_OFF,  STATE_OFF,  STATE_ON, STATE_ON, STATE_ON,   STATE_OFF },
      { STATE_OFF,  STATE_OFF,  STATE_ON, STATE_ON, STATE_ON,   STATE_ON },
      { STATE_OFF,  STATE_OFF,  STATE_ON, STATE_ON, STATE_ON,   STATE_BLINK } }
  }, {  // minus rail
    3,
    { 14.0, 16.0, 23.0 },
    {  // positions:  1 red, 1 green
      { STATE_BLINK,  STATE_OFF   },
      { STATE_ON,     STATE_OFF   },
      { STATE_OFF,    STATE_ON    },
      { STATE_OFF,    STATE_BLINK } } } };

int levels[NUM_RAILS];

// volts[0] holds plus rail, volts[1] holds minus rail but should be positive
float volts[NUM_RAILS];
int i = 0;

void playGame() {
  for( int rail=0; rail<NUM_RAILS; rail++ ) {
    levels[rail] = level( volts[rail], &LEDS_FOR_LEVELS[rail] );
    for( int i=0,j=BOTTOM_LED[rail]; j<=TOP_LED[rail]; j++,i++ )
      ledState[j] = LEDS_FOR_LEVELS[rail].leds_for_level[levels[rail]][i];
  }
}

int level( float v, struct leds_for_levels* leds_for_levels ) {
  // TODO fade in the top (or the next) segment of the mercury via PWM
  for( i=0; i<leds_for_levels->num_thresholds; i++ )
    if( v < leds_for_levels->thresholds[i] ) return i;
  return i;
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
  { 23.0, 15.0, { 2, 0,0,0, 0,0,  2, 0 } },
  { 24.5, 20.0, { 2, 2,0,0, 0,0,  0, 2 } },
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
