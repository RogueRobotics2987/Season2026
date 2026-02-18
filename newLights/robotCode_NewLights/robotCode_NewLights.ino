 
#include <Adafruit_NeoPixel.h>
#define PIN 8
#define NUMPIXELS 20
#define CHASE_DELAY 100
// Create a NeoPixel object
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB );


/*void setAllPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}*/
//function for the color chase


void setup() {
  // put your setup code here, to run once:
strip.begin();
strip.show();
}

void loop() {
// put your main code here, to run repeatedly:
// Blink green 3 times
  /*for (int i = 0; i < 3; i++) {
    setAllPixels(0, 150, 0); // Green ON
    delay(500);              // Wait 0.5 sec
    setAllPixels(0, 0, 0);   // OFF
    delay(500);              // Wait 0.5 sec
  }

  // Stop after 3 blinks
  while (true) {
    // Do nothing
  }*/

  /*for (int i = 0; i < 4; i++){
    setAllPixels(150, 0, 150);
    delay(500);
    setAllPixels(0,0,0);
    delay(500);
  }

  while(true){

  }*/


//setAllPixels (150, 0, 0);

//setAllPixels(150, 0, 150);

/*for (int g = 0; g < 4; g++){
setAllPixels(100, 0, 50);
delay(250);
set.pixwl
};
*/

  

}


