#include <FastLED.h>
#define DATA_PIN    3

#define LED_TYPE    WS2811
#define COLOR_ORDER RGB
#define NUM_LEDS    90
CRGB leds[NUM_LEDS];
#define FRAMES_PER_SECOND   144

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

uint8_t brightness = 255;
uint8_t curPattern = 0;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // 1 second delay for recovery
  delay(1000);
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // set master brightness control
  FastLED.setBrightness(brightness);
}

// Pattern codes from RoboRIO:
// '0': Idle
// '1': Blue
// '2': Red
// '3': Demo
// '4': Off

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { sinelon, blue, red, rainbow, black };

uint8_t gHue = 0; // rotating "base color" used by many of the patterns

void loop() {
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    // look for the next valid integer in the incoming serial stream:
    curPattern = Serial.parseInt();
    
    if (curPattern > ARRAY_SIZE(gPatterns)) {
      curPattern = ARRAY_SIZE(gPatterns) - 1;
    }
  }

  // Call the current pattern function once, updating the 'leds' array
  gPatterns[curPattern]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND); 

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy(leds, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV(gHue, 255, 192);
}

void blue() {
  for(int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CRGB::Blue;
  }
}

void red() {
  for(int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CRGB::Red;
  }
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
}

void black() {
  for(int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CRGB::Black;
  }
}
