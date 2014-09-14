#include <stdio.h>

int ledsState( float v, int rail );
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
  { 0, 27.0, 5 },
  { 0, 28.0, 6 },
  { 1, 21.0, 0 },
  { 1, 23.0, 1 },
  { 1, 24.5, 2 },
  { 1, 25.5, 2 },
  { 1, 26.5, 2 },
  { 1, 27.0, 2 },
  { 1, 28.0, 6 } };

int main( int arcg, char** argv ) {
  int failures = 0;
  struct test_case* test_case=test_cases;
  for( int i=0;
    i<sizeof(test_cases)/sizeof(*test_cases);
    test_case++, i++ ) {
    int level = ledsState( test_case->v, test_case->rail );
    printf( "ideal level %d %s actual level %d <= rail=%d, v=%f\n",
      test_case->ideal_level, test_case->ideal_level==level ? "=" : "!=", level,
      test_case->rail, test_case->v );
    if( test_case->ideal_level!=level ) failures++;
  }
  if( failures ) {
    printf( "%d failures\n", failures );
    return 1;
  } else
    return 0;
}




int i = 0;
#define NUM_LEVELS 6
#define NUM_LEDS 8 // Number of LED outputs
const float levelVolt[NUM_LEVELS] = { 22.0, 23.5, 24.8, 25.7, 26.7, 27.2 };

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
  #define LEVEL_HIGH_SAFE 5
  { 0,0, 1,1,1, 0,  0, 1 },
  #define LEVEL_OVER 6
  { 0,0, 1,1,1, 1,  1, 1 } };


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
      if( levelVolt[i] > v ) return LEVEL_LOW_SAFE+i-1;
      // since we got past LEVEL_OVER, we know levelVolt[NUM_LEVELS-1] >= v
    }
  else  // minus rail
    if( levelVolt[1] > v )
      return LEVEL_LOW_SAFE;
    else
      return LEVEL_LOW_SAFE+1;
}

