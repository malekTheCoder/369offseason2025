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
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
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
    private double DRIVE_POWER_VARIABLE = 1;
    private DistanceSensor distanceSensor;
    private boolean isTurning;
    private boolean foldUp;
    private RevColorSensorV3 colorSensor;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        // linearSlide = hardwareMap.get(DcMotor.class, "slide");
        isTurning=false; // is the robot currently turning?
        foldUp=false; // should the sampleArm servo fold up?

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
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
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {



        handleDriveTrain();
        handleClaw();
        handleClawRot();
        //handleLowRaise();
        handleArm();
        handleRotation();
        //handleSlide();

        telemetry.addData("backLeft position", backLeft.getCurrentPosition());
        telemetry.addData("frontLeft position", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight position", frontRight.getCurrentPosition());
        telemetry.addData("backRight position", backRight.getCurrentPosition());
        telemetry.addData("detected color", colorSensor.getLightDetected());
        //telemetry.addData("linearSlide position", linearSlide.getCurrentPosition());
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);
        telemetry.update();
    }

    private void handleDriveTrain() {
        // general movements
        if (gamepad1.dpad_up && distanceSensor.getDistance(DistanceUnit.INCH) > 10) {
            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
        }
        else if (gamepad1.dpad_down) {
            frontLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(-0.5);
        }
        else if (gamepad1.dpad_right) {
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
        }
        else if (gamepad1.dpad_left) {
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
        }
        else if (isTurning==false){
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    private void handleClaw() {
        if (gamepad1.left_bumper) {
            if (colorSensor.getNormalizedColors()==) {
                claw.setPosition(claw_max);
            }
        }
        else if (gamepad1.right_bumper) {
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
        if (gamepad1.right_trigger > 0.5 || distanceSensor.getDistance(DistanceUnit.INCH) < 24) {
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

    private void handleRotation() {
        // drive rotation
        if (gamepad1.left_stick_x>0.2){
            isTurning=true;
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
        }
        else if(gamepad1.left_stick_x<-0.2){
            isTurning=true;
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
        }
        else if (gamepad1.left_stick_x>-0.2 && gamepad1.left_stick_x<0.2){
            isTurning=false;
        }
    }
}
