package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.manualRobot;

@TeleOp(name = "Manual OpMode")
public class Main extends OpMode {
    private manualRobot robot;

    @Override
    public void init() {
        robot = new manualRobot(this);
        robot.init();
    }

    @Override
    public void loop() {
        robot.loop();
    }
}
