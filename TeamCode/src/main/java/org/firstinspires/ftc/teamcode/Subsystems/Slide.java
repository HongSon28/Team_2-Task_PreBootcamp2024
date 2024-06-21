package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.SLIDE.*;
import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    private DcMotor motor;
    private HardwareMap hardwareMap;
    public Slide(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "slideMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(SLIDE_SPEED);
        motor.setTargetPosition(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setState (boolean state) {
        if (state) {
            motor.setTargetPosition((int)(MAX_SIDE_POSITION * EXTEND_PERCENTAGE));
        } else {
            motor.setTargetPosition(0);
        }
    }
    public boolean isReached() {
        return (motor.getCurrentPosition() == (int)(MAX_SIDE_POSITION * EXTEND_PERCENTAGE));
    }
}
