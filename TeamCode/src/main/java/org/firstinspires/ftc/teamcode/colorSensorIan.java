package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="ianColorSensor")
public class colorSensorIan extends OpMode {
    private Servo claw;
    //private DcMotor linearSlide;
    private double claw_max;
    private double claw_min;
    private Servo clawRot;
    private double clawRot_max;
    private double clawRot_min;
    private Servo lowRaise;
    // private double lowRaise_max;
    // private double lowRaise_min;
    private Servo sampleArm;
    private double arm_max;
    private double arm_min;
    private RevColorSensorV3 colorSensor;
    private NormalizedRGBA colors;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");

        sampleArm = hardwareMap.get(Servo.class, "sampleArm");
        arm_max = 0.8;
        arm_min = 0.2;
        clawRot = hardwareMap.get(Servo.class, "rotation");
        clawRot_max = 0.8;
        clawRot_min = 0.2;
        //lowRaise = hardwareMap.get(Servo.class, "lowRaise");
        // lowRaise_max = 0.8;
        //lowRaise_min = 0.2;
        claw = hardwareMap.get(Servo.class, "claw");
        claw_max = 0.8;
        claw_min = 0.2;
        // linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        colors = colorSensor.getNormalizedColors();

        handleClaw();
        handleClawRot();
        //handleLowRaise();
        handleArm();
        //handleSlide();

        telemetry.addData("detected color", colorSensor.getLightDetected());
        //telemetry.addData("linearSlide position", linearSlide.getCurrentPosition());

        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);
        telemetry.update();


    }

    private void handleClaw() {
        if (gamepad1.right_bumper || colors.red>200) {
            claw.setPosition(claw_min);
        }
    }

    private void handleClawRot() {
        if (gamepad1.y) {
            clawRot.setPosition(clawRot_max);
        } else if (gamepad1.a) {
            clawRot.setPosition(clawRot_min);
        }
    }

    /*
    private void handleLowRaise() {

        if (gamepad1.b){
            lowRaise.setPosition(lowRaise_min); // interchangeable
        }
        else if (gamepad1.x){
            lowRaise.setPosition(lowRaise_max);
        }
    }
    */


    private void handleArm() {
        if (gamepad1.right_trigger > 0.5) {
            sampleArm.setPosition(arm_min); // 0.2
        }
        else if (gamepad1.left_trigger > 0.5) {
            sampleArm.setPosition(arm_max); // 0.8
        }
    }

    /*private void handleSlide() {
        if (gamepad1.right_stick_y < -0.3) {
            linearSlide.setPower(-0.5);
        }
        //else if(gamepad1.right_stick_y>0.3){
        //    linearSlide.setPower(0.5);
        //}
        else {
            linearSlide.setPower(0);
        }
    }*/
}
