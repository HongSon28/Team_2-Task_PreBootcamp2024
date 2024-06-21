package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.SLIDE_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Climber {
    private DcMotor motor;
    private Servo servo;
    private HardwareMap hardwareMap;
    public Climber(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "climberMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        servo = hardwareMap.get(Servo.class, "climberServo");
        servo.setPosition(0);
    }

    public void setServo(boolean state) {
        if (!state) servo.setPosition(0);
        else servo.setPosition(0.75);
    }

    public void setMotor(double power) {
        motor.setPower(power);
    }
}
