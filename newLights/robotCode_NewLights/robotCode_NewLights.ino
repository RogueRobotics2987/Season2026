 
#include <Adafruit_NeoPixel.h>
#define PIN 8
#define NUMPIXELS 20
//#define CHASE_DELAY 100
// Create a NeoPixel object
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB );

const int buttonPin = 4; // the number of the pushbutton pin
const int buttonPin2 = 5;
const int buttonPin3 = 6;

int buttonState = 0;
int buttonState3 = 0;
int buttonState2 = 0;




void Off(){
  strip.clear();
  strip.show();

}

void setAllPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
  strip.begin();
  strip.show();
  strip.clear();
  Serial.begin(9600);
}

void greenBlink(){

for (int i = 0; i < 3; i++) {
    setAllPixels(0, 150, 0); // Green ON
    delay(500);              // Wait 0.5 sec
    setAllPixels(0, 0, 0);   // OFF
    delay(500);              // Wait 0.5 sec
  }

}

void purpleBlink(){
  for (int i = 0; i < 4; i++){
    setAllPixels(150, 0, 150);
    delay(500);
    setAllPixels(0,0,0);
    delay(500);
  }

}

void red(){
  setAllPixels (150, 0, 0);
}

void purple(){
  setAllPixels(150, 0, 150);
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
buttonState = digitalRead(buttonPin); 
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3);
  Serial.print(buttonState);
  Serial.print(buttonState2);
  Serial.println(buttonState3);

  if (buttonState == 0 && buttonState2 == 0 && buttonState3 == 0) {
    Off();
  }
 else if (buttonState == 1 && buttonState2 == 0 && buttonState3 == 0) {
    greenBlink();
  }
  else if (buttonState == 0 && buttonState2 == 1 && buttonState3 == 0) {
    red();
  }
  else if (buttonState == 0 && buttonState2 == 0 && buttonState3 == 1) {
    purple();
  }
  else if (buttonState == 1 && buttonState2 == 1 && buttonState3 == 0) {
    purpleBlink();
  }
  Serial.println("waiting");
  delay(1000);
}


