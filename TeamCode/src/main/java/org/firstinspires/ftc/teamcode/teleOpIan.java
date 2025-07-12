package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Ian")
public class teleOpIan extends OpMode {
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private Servo claw;
    private double claw_max;
    private double claw_min;
    private Servo clawRot;
    private double clawRot_max;
    private double clawRot_min;
    private Servo lowRaise;
    private double lowRaise_max;
    private double lowRaise_min;
    private Servo arm;
    private double arm_max;
    private double arm_min;
    private double DRIVE_POWER_VARIABLE = 1;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        arm = hardwareMap.get(Servo.class, "arm");
        arm_max = 0.8;
        arm_min= 0.2;
        clawRot = hardwareMap.get(Servo.class, "clawRot");
        clawRot_max = 0.8;
        clawRot_min = 0.2;
        lowRaise = hardwareMap.get(Servo.class, "lowRaise");
        lowRaise_max = 0.8;
        lowRaise_min = 0.2;
        claw = hardwareMap.get(Servo.class, "claw");
        claw_max = 0.8;
        claw_min = 0.2;
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        handleDriveTrain();
        handleClaw();
        handleClawRot();
        handleLowRaise();
        handleArm();

        telemetry.addData("backLeft position", backLeft.getCurrentPosition());
        telemetry.addData("frontLeft position", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight position", frontRight.getCurrentPosition());
        telemetry.addData("backRight position", backRight.getCurrentPosition());
    }

    private void handleDriveTrain() {
        double var = 1;
        double StickX = gamepad1.left_stick_x;
        double StickY = gamepad1.left_stick_y;
        if (Math.abs(gamepad1.left_stick_x) > 0.2) {
            frontLeft.setPower(var * StickX);
            frontRight.setPower(-var * StickX);
            backLeft.setPower(-var * StickX);
            backRight.setPower(var * StickX);
        }
        if (Math.abs(gamepad1.left_stick_y) > 0.2) {
            frontLeft.setPower(var * StickY);
            frontRight.setPower(var * StickY);
            backLeft.setPower(var * StickY);
            backRight.setPower(var * StickY);
        }
    }

    private void handleClaw() {
        if (gamepad1.right_bumper) {
            claw.setPosition(claw_min);
        } else if (gamepad1.left_bumper) {
            claw.setPosition(claw_max);
        }
    }

    private void handleClawRot() {
        if (gamepad1.y){
            clawRot.setPosition(clawRot_max);
        }
        else if (gamepad1.a){
            clawRot.setPosition(clawRot_min);
        }
    }
    private void handleLowRaise() {
        if (gamepad1.b){
            lowRaise.setPosition(lowRaise_min); // interchangeable
        }
        else if (gamepad1.x){
            lowRaise.setPosition(lowRaise_max);
        }
    }
    private void handleArm() {
        if(gamepad1.dpad_up){
            arm.setPosition(arm_max);
        }
        else if(gamepad1.dpad_down){
            arm.setPosition(arm_min);
        }
    }
}