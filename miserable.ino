/*
  Continuous Servo Control with Random Speed and Direction
  ---------------------------------------------------------

  This Arduino sketch is designed to demonstrate the dynamic control of a continuous rotation 
  servo motor using a button. Continuous rotation servos, unlike standard servos, do not move 
  to specific angles but instead vary in speed and direction based on the signal sent. This 
  example toggles between a stopped state and a randomly chosen speed/direction state each time 
  the button is pressed.

  Components Required:
  - 1x Continuous Rotation Servo Motor
  - 1x Pushbutton Switch
  - 1x Arduino Board (e.g., Uno, Mega, Nano)
  - Jumper Wires
  - Breadboard (optional, for easier prototyping)

  Circuit Connections:
  - The servo motor's control wire is connected to digital pin 14 (A0) on the Arduino.
  - The pushbutton switch is connected between digital pin 15 (A1) and ground, with the internal 
    pull-up resistor enabled to read the button state.

  Libraries Used:
  - <Servo.h>: Included with the Arduino IDE, this library allows for easy control over servo motors.

  Code Overview:
  - Pin assignments are specified for both the servo and the button.
  - Servo positions (in this context, speed and direction values) are defined for different states, 
    with one state stopping the servo and the other dynamically chosen for varied speed/direction.
  - A `Servo` object named `notifier` is created for servo control.
  - In `setup()`, serial communication begins, the button pin is set with an internal pull-up resistor, 
    and the servo is attached to its control pin.
  - The `loop()` function checks the button state, updates the system's state, and controls the servo 
    accordingly, including selecting a random speed/direction when entering state 1.
  - The `determinState()` function handles the state change and the random selection for the servo's 
    behavior in state 1.

  How It Works:
  - Initially, the servo is in a stopped position (near the 94 angle, which is typically neutral for 
    continuous servos).
  - Upon pressing the button, the servo either stops (if moving) or starts moving in a randomly 
    selected speed and direction. This randomness is achieved by selecting a random value between 
    0 (maximum speed in one direction) and 180 (maximum speed in the opposite direction).
  - The current state and the selected random value (if applicable) are output to the serial monitor.

  Note:
  - Continuous rotation servos interpret the signal differently from standard servos: a value near 
    90 stops the servo, values decreasing from 90 increase speed in one direction, and values 
    increasing from 90 increase speed in the opposite direction.

  This sketch serves as an educational tool for understanding continuous rotation servo control, 
  incorporating randomness for dynamic behavior, and practicing basic input handling with Arduino.
*/


#include <Servo.h> // Include the Servo library to control servo motors.

// Pin assignments
int servoPin = 14; // Servo motor control pin connected to pin A0 on the Arduino.
int buttonPin = 15; // Button pin connected to digital pin A1 on the Arduino.

// Servo positions for different states
int state0angle = 94; // Value to stop the servo
int state1angle; // Chosen randomly each time. 0=max speed clockwise, 180=max speed counter-clockwise

// Current state of the system
bool currentState = 0; // The starting state of the system. (0 by default)

// Button state variables
bool buttonCurrent; // Stores the current read value of the button.
bool buttonPrev = true; // Stores the previous read value of the button to detect state changes.

// Servo object
Servo notifier; // Create a Servo object to control the servo motor.


void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 bits per second.
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as an input with an internal pull-up resistor.
  notifier.attach(servoPin); // Attach the servo motor to the servo pin.
 

}

void loop() {
  determineState(); // Determine the current state based on the button input.
  Serial.println(currentState); // Output the current state to the serial monitor.
  
  // Move the servo based on the current state
  switch (currentState) {
    case 0:
      notifier.write(state0angle); // Set the servo to the angle for state 0.
      break;
    case 1:
      notifier.write(state1angle); // Set the servo to the angle for state 1.
      break;
  }
}

// Function to determine the current state based on button input
void determineState() {
  buttonCurrent = digitalRead(buttonPin); // Read the current state of the button.

  // If the button state has changed from not pressed to pressed
  if(buttonCurrent == 1 && buttonPrev == 0) {
    currentState = !currentState; // Toggle the current state.
    state1angle = random(0,180); //choose a random target angle
  }

  buttonPrev = buttonCurrent; // Update the previous button state for the next loop iteration.
}



