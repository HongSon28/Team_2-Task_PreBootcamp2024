package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.START_POSE;
import static org.firstinspires.ftc.teamcode.Constants.AUTONOMOUS.*;
import static org.firstinspires.ftc.teamcode.Constants.PID.*;
import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Slide;

public class autonomousRobot {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private IMU imu;
    private Odometry odometry;
    private Slide slide;
    private Climber climber;
    private boolean servoState = false;
    private boolean Climbed = false;
    private boolean ReachedParkingPosition = false;
    private boolean auto = true;
    private PIDController distanceController;
    private PIDController angleController;

    public autonomousRobot (OpMode opMode) {
        drivetrain = new Drivetrain(opMode);
        slide = new Slide(opMode);
        climber = new Climber(opMode);
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        gamepad = opMode.gamepad1;
    }

    public void init() {
        drivetrain.init();
        slide.init();
        climber.init();

        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(imuParams);
        odometry = new Odometry(START_POSE);

        distanceController = new PIDController(DRIVE_X_PID, -DRIVE_SPEED, DRIVE_SPEED);
        angleController = new PIDController(DRIVE_ROT_PID, -DRIVE_SPEED, DRIVE_SPEED);
    }
    public void loop() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        odometry.update(drivetrain.getLeft(), drivetrain.getRight(),new Rotation2d(angle));

        if (auto) {
            if (odometry.isReached()) {
                if (!Climbed) {
                    slide.setState(true);
                    if (slide.isReached()) Climbed = true;
                } else if (!ReachedParkingPosition) {
                    drivetrain.setDrivePower(DRIVE_SPEED, DRIVE_SPEED);
                    if (odometry.getDist() >= RETRACTING_POSITION) slide.setState(false);
                    if (odometry.getDist() >= PARKING_POSITION) ReachedParkingPosition = true;
                }
            } else {
                double f = distanceController.control(odometry.getDist());
                double t = angleController.control(odometry.get_rot());
                drivetrain.setDrivePower(f + t, f - t);
            }
            if (gamepad.left_bumper) auto = false; //Abort autonomous mode
        } else {
            drivetrain.setDrivePower(-gamepad.left_stick_y,-gamepad.right_stick_y);

            if (gamepad.right_bumper) servoState = true;
            climber.setServo(servoState);

            climber.setMotor(gamepad.right_trigger);
        }
    }
}
