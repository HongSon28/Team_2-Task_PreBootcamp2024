package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hooker {
    private Servo servoLeft, servoRight;
    private HardwareMap hardwareMap;
    private boolean stateLeft = false, stateRight = false;
    public Hooker(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }
    public void init() {
        servoLeft = hardwareMap.get(Servo.class, "hookerServoLeft");
        servoLeft.setDirection(Servo.Direction.REVERSE);
        servoLeft.setPosition(0);

        servoRight = hardwareMap.get(Servo.class, "hookerServoRight");
        servoRight.setPosition(0);
    }

    public void setStateLeft() {
        stateLeft = true;
    }

    public void setStateRight() {
        stateRight = true;
    }

    public void setServo() {
        if (!stateLeft) servoLeft.setPosition(0);
        else servoLeft.setPosition(0.66);
        if (!stateRight) servoRight.setPosition(0);
        else servoRight.setPosition(0.66);
    }
}
