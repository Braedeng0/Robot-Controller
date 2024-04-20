package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideArm {
    private final Telemetry telemetry;

    // Motors
    private final DcMotor slideMotor;

    //  Max position
    private final int maxPosition = -3000;

    public SlideArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.slideMotor = hardwareMap.get(DcMotor.class, "slideArm");

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPosition(double position, double power) {
        // Convert position (0-100) to encoder ticks
        int targetPosition = (int) (position / 100 * maxPosition);

        slideMotor.setTargetPosition(targetPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void breakOnStop(boolean brake) {
        if (brake) {
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}
