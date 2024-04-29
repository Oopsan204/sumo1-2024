#include "AML_MPUSensor.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Constants for PID control
#define Kp 1.0  // Proportional gain
#define Ki 0.1  // Integral gain
#define Kd 0.01 // Derivative gain

// Initialize variables
double target_angle = 0.0; // Target angle (usually 0 degrees for straight line)
double integral = 0.0;
double previous_error = 0.0;
double speed1;
double speed2;

// Function to compute PID control output
double compute_pid_output(double current_angle)
{
    double error = target_angle - current_angle;
    integral += error;
    double derivative = error - previous_error;
    double output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    return output;
}

// Simulate reading angle from MPU6050 (replace with actual sensor data)
double read_mpu6050_angle()
{
   
    // Simulate reading angle (replace with actual sensor data)
    return 10.0; // Example: 10 degrees
}

int16_t math_PID(double Angle_PID)
{
    double current_angle;
    double pid_output;

        current_angle = Angle_PID;
        pid_output = compute_pid_output(current_angle);

        // Apply PID output to motor control (adjust motor speeds)
        speed2 = 100.0 + pid_output;
        speed1 = 100.0 - pid_output;

    
}
