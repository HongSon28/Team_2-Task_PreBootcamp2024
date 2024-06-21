package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.SlidePositionCalibration;

@TeleOp(name = "Slide Position Calibration")
public class Calibration extends OpMode {
    private SlidePositionCalibration robot;
    @Override
    public void init() {
        robot = new SlidePositionCalibration(this);
        robot.init();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.loop();
    }
}
