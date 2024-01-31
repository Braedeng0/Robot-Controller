package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AirplaneLauncher {
    private static final double airplaneReady = .175;
    private static final double airplaneLaunch = 1;
    private boolean airplaneToggle = false;
    private Servo airplaneServo;

    public AirplaneLauncher(HardwareMap hardwareMap) {
        airplaneServo = hardwareMap.servo.get("airplane_servo");
        ready();
    }

    public void ready() {airplaneServo.setPosition(airplaneReady);}

    public void launch() {airplaneServo.setPosition(airplaneLaunch);}

    public void toggle() {if (!airplaneToggle) launch(); else ready();}
}
