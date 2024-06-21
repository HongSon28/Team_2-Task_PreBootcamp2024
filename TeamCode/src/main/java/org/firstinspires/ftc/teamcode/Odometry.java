package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.*;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.qualcomm.robotcore.hardware.IMU;

public class Odometry {
    private Pose2d pose;
    private Rotation2d lastAngle;
    private double lastLeft, lastRight;
    public Odometry(Pose2d initialPose) {
        pose = initialPose;
        lastLeft = lastRight = 0;
        lastAngle = new Rotation2d(0);
    }

    public void update(double currentLeft, double currentRight, Rotation2d currentAngle) {
        double deltaLeft = currentLeft - lastLeft;
        double deltaRight = currentRight - lastRight;
        lastLeft = deltaLeft;
        lastRight = deltaRight;
        double dx = (deltaLeft + deltaRight) / 2;
        Rotation2d dTheta = currentAngle.minus(lastAngle);
        Twist2d twist = new Twist2d(dx, 0,dTheta.getRadians());
        pose = pose.exp(twist);
    }

    public double get_dx () {
        return TARGET_X - pose.getX();
    }

    public double get_dy() {
        return TARGET_Y - pose.getY();
    }

    public double get_rot() {
        return pose.getHeading();
    }

    public double getAngle() {
        return Math.atan2(get_dy(),get_dx());
    }
    public double getDist() {
        return Math.hypot(get_dx(),get_dy());
    }

    public boolean isReached() {
        double x = get_dx(), y = get_dy(), rot = get_rot();
        return (Math.abs(x - TARGET_X) <= X_TOLERANT && Math.abs(y - TARGET_Y) <= Y_TOLERANT && Math.abs(rot) <= ROT_TOLERANT);
    }
}
