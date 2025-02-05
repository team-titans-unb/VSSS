/**********************************************************************************************/
/*                                                                                            */
/*  File: main.ino                                                                            */
/*  Author: Luiz Felipe                                                                       */
/*  Description: This file implements the control of a robot using a PS3 controller.          */
/*               It receives analog data from the controller to control the robot's motors,   */
/*               allowing forward, backward movements, and smooth rotations to the left and   */
/*               right. The motors are controlled by adjusting speeds based on the analog     */
/*               values received, providing smooth and responsive movements.                  */
/*  Created: 2023/02/07 by Luiz F.                                                            */
/*  Updated: 2023/03/31 by Luiz F.                                                            */
/*  Contact: luizcn123@hotmail.com                                                            */
/*  Location: DF, BRAZIL                                                                      */
/*  All rights reserved                                                                       */
/**********************************************************************************************/
#include "src/behavior.h"
#include "src/led_rgb.h"
#include "src/motor.h"
#include <Ps3Controller.h>

Motor motor1(MOTOR1_PIN_R, MOTOR1_PIN_L, PWM_MOTOR1_CHANNEL_R, PWM_MOTOR1_CHANNEL_L);
Motor motor2(MOTOR2_PIN_R, MOTOR2_PIN_L, PWM_MOTOR2_CHANNEL_R, PWM_MOTOR2_CHANNEL_L);

// Variables to sticks ly and rx from ps3 controller.
int leftY;
int rightX;

void notify() {
    leftY = Ps3.data.analog.stick.ly; 
    rightX = Ps3.data.analog.stick.rx;
    Serial.print("\nVelocidades");
    Serial.print(leftY);
    Serial.println(rightX);
    int speedY = map(leftY, 128, -128, -255, 255); // Mapping analog values to speed intervals
    int speedX = map(rightX, -128, 128, -255, 255); // Mapping analog values to direction intervals
    float ajuste;
    if ((speedY >= 40 ) && (speedX >= 40)){
        // makes the robot move straight and turn smoothly to the right
        Serial.println("Go foward turn right");
        ajuste = speedY - (speedX*speedY/255);
        motor1.moveForward((int)ajuste -40, -1);
        motor2.moveForward( -speedY, 1);
    } else if ((speedY >= 40 ) && (speedX <= -40)){
        // makes the robot move forward and turn smoothly to the left
        Serial.println("Go foward turn left");
        ajuste = speedY - (-speedX*speedY/255);
        motor1.moveForward(speedY -40, -1);
        motor2.moveForward((int)ajuste -40, 1);
    } else if ((speedY <= -40 ) && (speedX >= 40)){
        // makes the robot move backwards and gently turn to the left
        Serial.println("Go backward turn left");
        ajuste = -speedY - (-speedX*speedY/255);
        motor1.moveForward((int)ajuste - 40, 1);
        motor2.moveForward(-speedY -40, -1);
    } else if ((speedY <= -40 ) && (speedX <= -40)){
        // makes the robot move backwards and gently turn to the right
        Serial.println("Go backward turn right");
        ajuste = -speedY - (-speedX*(-speedY)/255);
        motor1.moveForward(-speedY -40, 1);
        motor2.moveForward((int)ajuste -40, -1);
    } else if ((speedY >= 40) && (speedX > -40) && (speedX < 40)){
        // makes the robot go straight
        Serial.println("Go forward");
        motor1.moveForward(speedY -40, 1);
        motor2.moveForward(speedY -40, -1);
    } else if ((speedY <= -40) && (speedX > -40) && (speedX < 40)){
        //makes the robot go backwards
        Serial.println("Go backward");
        motor1.moveForward(speedY -40, -1);
        motor2.moveForward(speedY -40, 1);
    } else{
        // mmakes the robot to stop
        Serial.println("Stop");
        motor1.stop();
        motor2.stop();
    }
}

// Function to execute when controller is connected
void onConnect() {
    // Print to Serial Monitor
    Serial.println("Connected.");
}

void setup() {
    Serial.begin(19200); // Initialize serial communication
    Ps3.attach(notify); // Attach the notification function to the PS3 controller
    Ps3.attachOnConnect(onConnect); // Attach the connection function to the PS3 controller
    Ps3.begin(CONTROLS_MAC_ADDRESS); // Begin PS3 controller communication
    Serial.println("Ready."); // Print "Ready." to serial monitor
}

void loop() {}
