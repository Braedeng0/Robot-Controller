package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AirplaneLauncher {
    private static final double airplaneReady = .175;
    private static final double airplaneLaunch = 1;
    private static Servo airplaneServo;

    public AirplaneLauncher() {
        airplaneServo = hardwareMap.Servo.get("airplane_servo");

    }
}
