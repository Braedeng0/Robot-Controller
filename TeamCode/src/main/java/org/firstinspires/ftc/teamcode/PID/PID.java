package org.firstinspires.ftc.teamcode.PID;

public class PID {
    public double kP, kI, kD;
    public double[] errorBuffer;
    public int windowSize;
    public double integral, prevError;

    public PID(double kP, double kI, double kD, double initialError, int windowSize) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.windowSize = windowSize;

        errorBuffer = new double[windowSize];
        integral = 0;
        prevError = initialError;

        for (int i = 0; i < windowSize; i++) {
            errorBuffer[i] = initialError;
        }
    }

    public double calculate (double current, double target) {
        double error = target - current;

        // Moving average filter to smooth out the error
        for (int i = 0; i < windowSize - 1; i++) {
            errorBuffer[i] = errorBuffer[i + 1];
        }
        errorBuffer[windowSize - 1] = error;

        error = 0;
        for (int i = 0; i < windowSize; i++) {
            error += errorBuffer[i];
        }
        error /= windowSize;

        // Calculate integral and derivative
        integral += error;
        double derivative = error - prevError;

        // Calculate move
        double move = kP * error + kI * integral + kD * derivative;

        // Update previous error
        prevError = error;

        // Return the move
        return move;
    }

    public double getError() {
        return errorBuffer[windowSize - 1];
    }
}
