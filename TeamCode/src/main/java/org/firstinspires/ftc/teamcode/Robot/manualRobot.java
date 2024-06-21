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
import org.firstinspires.ftc.teamcode.Subsystems.Hooker;
import org.firstinspires.ftc.teamcode.Subsystems.Slide;

public class manualRobot {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Climber climber;
    private Hooker hooker;
    private boolean servoState = false;

    public manualRobot (OpMode opMode) {
        gamepad = opMode.gamepad1;
        drivetrain = new Drivetrain(opMode);
        climber = new Climber(opMode);
        hooker = new Hooker(opMode);
    }

    public void init() {
        drivetrain.init();
        climber.init();
        hooker.init();
    }

    public void loop() {
        drivetrain.setDrivePower(-gamepad.left_stick_y,-gamepad.right_stick_y);

        if (gamepad.circle) climber.setState();

        if (gamepad.right_trigger != 0) climber.setMotor(gamepad.right_trigger);
        else if (gamepad.left_trigger != 0) climber.setMotor(-gamepad.left_trigger);
        else climber.setMotor(0);

        if (gamepad.dpad_left) hooker.setStateLeft();
        if (gamepad.dpad_right) hooker.setStateRight();
    }
}
