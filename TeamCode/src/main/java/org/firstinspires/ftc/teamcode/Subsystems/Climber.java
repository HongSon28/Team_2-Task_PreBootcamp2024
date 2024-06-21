package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.SLIDE_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Climber {
    private DcMotor motor1, motor2;
    private Servo servo;
    private HardwareMap hardwareMap;
    private boolean state = false;
    public Climber(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "climberMotor1");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setPower(0);

        motor2 = hardwareMap.get(DcMotor.class, "climberMotor2");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setPower(0);

        servo = hardwareMap.get(Servo.class, "climberServo");
        servo.setPosition(0);
    }

    public void setState() {
        state = true;
    }

    public void setServo() {
        if (!state) servo.setPosition(0);
        else servo.setPosition(0.33);
    }

    public void setMotor(double power) {
        motor1.setPower(power);
    }
}
