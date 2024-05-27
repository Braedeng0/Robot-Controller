package org.firstinspires.ftc.teamcode.PID;

public class PID {
    public double kP, kI, kD;
    public double integral, prevError;
    public boolean haltIntegral;

    public PID (double kP, double kI, double kD, double initialError) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        integral = 0;
        prevError = initialError;
        haltIntegral = false;
    }

    public double calculate (double current, double target) {
        // Calculate error
        double error = target - current;

        // Calculate derivative
        double derivative = error - prevError;

        // Update integral
        if (!haltIntegral) {
            integral += error;
        }

        // Calculate move
        double move = kP * error + kI * integral + kD * derivative;

        // Update previous error
        prevError = error;

        // Normalize move to be between -1 and 1
        // If the system is moving at 100%, stop the integral to prevent windup
        if (move > 1) {
            move = 1;
            haltIntegral = true;
        } else if (move < -1) {
            move = -1;
            haltIntegral = true;
        } else {
            haltIntegral = false;
        }

        // Return the move
        return move;
    }

    public double getError() {
        return prevError;
    }
}

