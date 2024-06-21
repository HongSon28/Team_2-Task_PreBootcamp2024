package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private HardwareMap hardwareMap;
    private DcMotorEx left, right;
    public Drivetrain (OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }
    public void init() {
        left = hardwareMap.get(DcMotorEx.class,"leftMotor");
        right = hardwareMap.get(DcMotorEx.class,"rightMotor");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDrivePower(double leftPower, double rightPower) {
        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    public double getLeft() {
        return left.getCurrentPosition() * TO_INCH_LEFT;
    }
    public double getRight() {
        return right.getCurrentPosition() * TO_INCH_RIGHT;
    }
}
