#include <stdio.h>

int level( float v, int rail );
int main( int arcg, char** argv );

struct test_case {
  int rail;
  float v;
  int ideal_level;
};
struct test_case test_cases[] = {
  { 0, 21.0, 0 },
  { 0, 23.0, 1 },
  { 0, 24.5, 2 },
  { 0, 25.5, 3 },
  { 0, 26.5, 4 },
  { 0, 26.9, 5 },
  { 0, 27.1, 6 },
  { 0, 28.0, 7 },
  { 1, 13.5, 0 },
  { 1, 15.0, 1 },
  { 1, 20.0, 2 },
  { 1, 22.5, 3 },
  { 1, 24.0, 7 } };

int main( int arcg, char** argv ) {
  int failures = 0;
  struct test_case* test_case=test_cases;
  for( int i=0;
    i<sizeof(test_cases)/sizeof(*test_cases);
    test_case++, i++ ) {
    int actual_level = level( test_case->v, test_case->rail );
    printf( "ideal level %d %s actual level %d <= rail=%d, v=%f\n",
      test_case->ideal_level, test_case->ideal_level==actual_level ? "=" : "!=", actual_level,
      test_case->rail, test_case->v );
    if( test_case->ideal_level!=actual_level ) failures++;
  }
  if( failures ) {
    printf( "%d failures\n", failures );
    return 1;
  } else
    return 0;
}


// test infrastructure stuff above
// real code below


#define NUM_RAILS 2
#define NUM_LEDS 8 // Number of LED outputs

#define STATE_OFF 0
#define STATE_BLINK 1
#define STATE_BLINKFAST 3
#define STATE_ON 2

#define NUM_THRESHOLDS 7
const float levelVolts[NUM_RAILS][NUM_THRESHOLDS] = {
  { 22.0, 23.5, 24.8, 25.7, 26.7, 27.0, 27.2 },
  { 14.0, 16.0, 22.0, 23.0, 23.0, 23.0, 23.0 } };
// a visually compact table of whether each led should be on/blinking vs off
// positions:  big pedalometer: 2 red, 3 green, 1 white,  side lights: 1 red, 1 green
const int LEDS_FOR_LEVEL[NUM_THRESHOLDS+1][NUM_LEDS] = {
  { STATE_BLINK,STATE_OFF,   STATE_OFF,  STATE_OFF,  STATE_OFF,   STATE_OFF,          STATE_BLINK, STATE_OFF   },
  { STATE_ON,   STATE_OFF,   STATE_OFF,  STATE_OFF,  STATE_OFF,   STATE_OFF,          STATE_ON,    STATE_OFF   },
  { STATE_ON,   STATE_ON,    STATE_OFF,  STATE_OFF,  STATE_OFF,   STATE_OFF,          STATE_OFF,   STATE_ON    },
  { STATE_OFF,  STATE_OFF,   STATE_ON,   STATE_OFF,  STATE_OFF,   STATE_OFF,          STATE_OFF,   STATE_ON    },
  { STATE_OFF,  STATE_OFF,   STATE_ON,   STATE_ON,   STATE_OFF,   STATE_OFF,          STATE_OFF,   STATE_ON    },
  { STATE_OFF,  STATE_OFF,   STATE_ON,   STATE_ON,   STATE_ON,    STATE_OFF,          STATE_OFF,   STATE_ON    },
  { STATE_OFF,  STATE_OFF,   STATE_ON,   STATE_ON,   STATE_ON,    STATE_ON,           STATE_OFF,   STATE_BLINK },
  { STATE_OFF,  STATE_OFF,   STATE_ON,   STATE_ON,   STATE_ON,    STATE_BLINK,        STATE_OFF,   STATE_BLINK } };
int i = 0;


int level( float v, int rail ) {
  // TODO fade in the top (or the next) segment of the mercury via PWM
  for( i=0; i<NUM_THRESHOLDS; i++ )
    if( v < levelVolts[rail][i] ) return i;
  return NUM_THRESHOLDS;
}
