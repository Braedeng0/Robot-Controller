package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AirplaneLauncher {
    private static final double airplaneReady = .175;
    private static final double airplaneLaunch = 1;
    private Servo airplaneServo;

    public AirplaneLauncher(HardwareMap hardwareMap) {
        airplaneServo = hardwareMap.servo.get("airplane_servo");
    }

    public void ready() {airplaneServo.setPosition(airplaneReady);}

    public void launch() {airplaneServo.setPosition(airplaneLaunch);}

    public void toggle() {
        if (airplaneServo.getPosition() == airplaneReady){
            airplaneServo.setPosition(airplaneLaunch);
        } else {
            airplaneServo.setPosition(airplaneReady);
        }
    }
}
