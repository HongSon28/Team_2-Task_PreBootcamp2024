package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.autonomousRobot;

@TeleOp(name = "Autonomous OpMode")
public class Autonomous extends OpMode {
    private autonomousRobot robot;

    @Override
    public void init() {
        robot = new autonomousRobot(this);
        robot.init();
    }

    @Override
    public void loop() {
        robot.loop();
    }
}
