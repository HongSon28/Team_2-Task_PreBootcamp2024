package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.*;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Slide;

public class manualRobot {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Climber climber;
    private boolean servoState = false;

    public manualRobot (OpMode opMode) {
        gamepad = opMode.gamepad1;
        drivetrain = new Drivetrain(opMode);
        climber = new Climber(opMode);
    }

    public void init() {
        drivetrain.init();
        climber.init();
    }

    public void loop() {
        drivetrain.setDrivePower(-gamepad.left_stick_y,-gamepad.right_stick_y);

        if (gamepad.right_bumper) servoState = true;
        climber.setServo(servoState);

        climber.setMotor(gamepad.right_trigger);
    }
}
