package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class Constants {
    public static final class SLIDE {
        public static final int MAX_SIDE_POSITION = 5000; //Encoder position of the slide (need calibration)
        public static final double EXTEND_PERCENTAGE = 0.75;
    }
    public static final class SPEED {
        public static final double SLIDE_SPEED = 1;
        public static final double DRIVE_SPEED = 0.25;
    }
    public static final class ODOMETRY {
        //TODO: Change these values later
        public static final Pose2d START_POSE = new Pose2d(0,0,new Rotation2d(0));
        public static final double TO_INCH_LEFT = 1;
        public static final double TO_INCH_RIGHT = 1;
        public static final double TARGET_X = 100;
        public static final double TARGET_Y = 0;
        public static final double X_TOLERANT = 0.1;
        public static final double Y_TOLERANT = 0.1;
        public static final double ROT_TOLERANT = 1;
    }
    public static final class PID {
        //TODO: Change these values later
        public static PIDCoefficients DRIVE_X_PID = new PIDCoefficients(0.02, 0.0, 0.0);
        public static PIDCoefficients DRIVE_ROT_PID = new PIDCoefficients(0.01, 0.0, 0.0);
    }
    public static final class AUTONOMOUS {
        public static double RETRACTING_POSITION = 15;
        public static double PARKING_POSITION = 30;
    }
}
