package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AirplaneLauncher {
    private static final double airplaneLaunch = 0.175;
    private static final double airplaneReady = 1;
    private Servo airplaneServo;

    public AirplaneLauncher(HardwareMap hardwareMap) {
        airplaneServo = hardwareMap.servo.get("airplaneLauncher");
        ready();
    }

    public void ready() {airplaneServo.setPosition(airplaneReady);}

    public void launch() {airplaneServo.setPosition(airplaneLaunch);}
}
