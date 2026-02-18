const int buttonPin = 4; // the number of the pushbutton pin
const int buttonPin2 = 5;
const int buttonPin3 = 6;




void setup() {
  // put your setup code here, to run once:
  pinMode(buttonPin, OUTPUT);
  pinMode(buttonPin2, OUTPUT);
  pinMode(buttonPin3, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(buttonPin, 1);
digitalWrite(buttonPin2, 1);
digitalWrite(buttonPin3, 0);
delay(500);


}
