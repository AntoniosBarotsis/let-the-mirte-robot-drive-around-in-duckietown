/*
 * Download the Adafruit NeoPixel library using the built-in library manager as follows:
 * - Press "Tools" in the menu bar.
 * - Press "Manage Libraries".
 * - Search for "Adafruit NeoPixel".
 * - Scroll down until you see the library with the name "Adafruit NeoPixel".
 * - Press install on this library.
 * - After installation of the library, you might have to restart the Arduino IDE for the changes to take place.
 */
#include <Adafruit_NeoPixel.h>

// -------------------- START of user defined variables --------------------

/* 
 * Set the VIN, DATA and GND pins below.
 * If you want to use a GPIO pin for VIN, set USE_GPIO_VIN to true and set VIN_PIN to the corresponding pin number, otherwise set USE_GPIO_VIN to false.
 * If you want to use a GPIO pin for GND, set USE_GPIO_GND to true and set GND_PIN to the corresponding pin number, otherwise set USE_GPIO_GND to false.
*/
#define USE_GPIO_VIN  true
#define USE_GPIO_GND  true
#define VIN_PIN       5
#define DATA_PIN      6
#define GND_PIN       7

/*
 * Set which sides of the traffic light you want to use, for each of the 4 sides fill in true to use that side, false otherwhise.
 * The order is from SIDE_0 (which corresponds to the first RGB LED in the chain), to SIDE_3 (which corresponds to the last RGB LED in the chain).
 * The sides which aren't used are set to red if UNUSED_SIDE_OFF is set to false, otherwise they won't light up.
 */
bool useSide[4] = {true, true, true, true};
#define UNUSED_SIDES_OFF   true

/*
 * The timing for one side of the traffic light can be defined below.
 * So these times are for the side that is green in the current cycle.
 * This means that the other sides will show a red light for (GREEN_TIME + ORANGE_TIME + RED_TIME) milliseconds.
 * The order in which the colors that are shown are: green, orange, red.
 * 
 * GREEN_TIME   is the time (in milliseconds) to set one side of the traffic light to green.
 * ORANGE_TIME  is the time (in milliseconds) to set one side the traffic light to orange.
 * RED_TIME     is the time (in milliseconds) to set the traffic light to red.
 */
#define GREEN_TIME    10000
#define ORANGE_TIME   2000
#define RED_TIME      5000

/*
 * The brightness of the Neopixels can be defined below.
 * TOTAL_BRIGHTNESS is the brightness of the whole traffic light, use this to change the maximum brightness of all colors at the same time.
 * RGB_X        is for setting the RGB colors of GREEN, ORANGE and RED respectively.
 * Note that all the values of TOTAL_BRIGHTNESS and RGB_X should be in the range [0, 255] both including.
 *  
 * RGB_GREEN    is the RGB color of the green light.
 * RGB_ORANGE   is the RGB color of the orange light.
 * RGB_RED      is the RGB color of the red light.
 */
#define TOTAL_BRIGHTNESS  20
#define RGB_GREEN       pixels.Color(0, 255, 0)
#define RGB_ORANGE      pixels.Color(127, 127, 0)
#define RGB_RED         pixels.Color(255, 0, 0)

// --------------------- END of user defined variables ---------------------

// ---------------- START of check of user defined variables ---------------

#if (TOTAL_BRIGHTNESS < 0 || TOTAL_BRIGHTNESS > 255)
  #error "TOTAL_BRIGHTNESS should be in the range [0, 255] both including."
#endif

// ----------------- END of check of user defined variables ----------------

Adafruit_NeoPixel pixels(4, DATA_PIN, NEO_GRB + NEO_KHZ800);

enum COLOR {
  GREEN   = 0,
  ORANGE  = 1,
  RED     = 2
};

enum SIDE {
  SIDE_0 = 0,
  SIDE_1 = 1,
  SIDE_2 = 2,
  SIDE_3 = 3,
};

struct STATE {
  COLOR       color_enum;
  uint32_t    color;
  long        waitTime;
  STATE       *next;
};

STATE redState = {RED, RGB_RED, RED_TIME, NULL};
STATE orangeState = {ORANGE, RGB_ORANGE, ORANGE_TIME, &redState};
STATE greenState = {GREEN, RGB_GREEN, GREEN_TIME, &orangeState};

SIDE currentSide = SIDE_0;
STATE currentState = greenState;

void setup() {
  #if USE_GPIO_GND
    pinMode(GND_PIN, OUTPUT);
    digitalWrite(GND_PIN, LOW);
  #endif
  
  #if USE_GPIO_VIN
    pinMode(VIN_PIN, OUTPUT);
    digitalWrite(VIN_PIN, HIGH);
  #endif

  pixels.begin();
  pixels.setBrightness(TOTAL_BRIGHTNESS);

  redState.next = &greenState;

  calculateNextSide();
}

void loop() {
  pixels.clear(); // Set all sides off

  for(int side = 0; side < 4; side++) {
    setColor(side);
  }
  
  pixels.show();  // Show the calculated colors

  delay(currentState.waitTime);

  calculateNextState();
}

/*
 * Sets the color of the provided side
 */
void setColor(int side) {
  if (currentSide == side) {
    pixels.setPixelColor(side, currentState.color);
  } else if (useSide[side] || !UNUSED_SIDES_OFF) {
    pixels.setPixelColor(side, RGB_RED);
  }
}

/*
 * Calculates the next state
 */
void calculateNextState() {
  if (currentState.color_enum == RED) {
    // We need to move to the next side
    calculateNextSide();
  }
  
  currentState = *currentState.next;
}

/*
 * Calculates the next side to be green
 */
void calculateNextSide() {
  currentSide = ((currentSide+1) % 4);
  if (!useSide[currentSide]) {
    calculateNextSide();
  }
}