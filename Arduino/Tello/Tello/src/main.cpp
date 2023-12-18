#include <Arduino.h>
#include <Tello.h>
#include <Kalman.h>

Tello tello; 


// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(115200);
  delay(5);
  Serial.println("Tello Flight Controller");
  Serial.println("Lennard Jönsson | Victor Kröger");

  tello.init();
}

void loop() {
  tello.takeoff();
  delay(5000);

  
  
  }

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}