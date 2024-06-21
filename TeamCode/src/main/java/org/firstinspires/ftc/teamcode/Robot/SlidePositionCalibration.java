package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlidePositionCalibration {
    private DcMotor slide;
    private Gamepad gamepad;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public SlidePositionCalibration(OpMode opMode) {
        gamepad = opMode.gamepad1;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        slide = hardwareMap.get(DcMotor.class,"slideMotor");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop() {
        slide.setPower(-gamepad.left_stick_y);
        telemetry.addData("Current slide position: ", slide.getCurrentPosition());
        telemetry.update();
    }
}
