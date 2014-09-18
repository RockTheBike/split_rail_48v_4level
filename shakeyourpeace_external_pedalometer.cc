#include <stdio.h>

int level( float v, int rail );
int main( int arcg, char** argv );


// real code below

#define NUM_RAILS 2
#define MAX_LEVELS 8
#define MAX_LEDS_PER_RAIL 6
#define NUM_LEDS 8 // Number of LED outputs

#define STATE_OFF 0
#define STATE_BLINK 1
#define STATE_BLINKFAST 3
#define STATE_ON 2

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
int i = 0;


int level( float v, struct leds_for_levels* leds_for_levels ) {
  // TODO fade in the top (or the next) segment of the mercury via PWM
  for( i=0; i<leds_for_levels->num_thresholds; i++ )
    if( v < leds_for_levels->thresholds[i] ) return i;
  return i;
}


// real code above
// test infrastructure stuff below


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
  { 1, 22.5, 2 },
  { 1, 24.0, 3 } };

int main( int arcg, char** argv ) {
  int failures = 0;
  struct test_case* test_case=test_cases;
  for( int i=0;
    i<sizeof(test_cases)/sizeof(*test_cases);
    test_case++, i++ ) {
    int actual_level = level( test_case->v, &LEDS_FOR_LEVELS[test_case->rail] );
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
