package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "basic teleop with slide testing")
public class eTeleopSlide extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor slide;
    private Servo claw;
    private Servo rotation;
    private Servo arm;
    //private Servo tower;
    private DistanceSensor distance;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        slide = hardwareMap.get(DcMotor.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        rotation = hardwareMap.get(Servo.class, "rotation");
        arm = hardwareMap.get(Servo.class, "sampleArm");
        //tower = hardwareMap.get(Servo.class, "tower");

        distance = hardwareMap.get(DistanceSensor.class, "distance");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        drivetrain();
        intake();


        telemetry.addData("frontLeft position", frontLeft.getCurrentPosition())
        telemetry.addData("frontRight position", frontRight.getCurrentPosition());
        telemetry.addData("slide position", slide.getCurrentPosition());
    }
    private void drivetrain(){
        frontLeft.setPower(-1*gamepad1.left_stick_x);
        frontRight.setPower(1*gamepad1.left_stick_x);

        frontLeft.setPower(1*gamepad1.left_stick_y);
        frontRight.setPower(1*gamepad1.left_stick_y);

        frontLeft.setPower(-1*gamepad1.right_stick_x);
        frontRight.setPower(1*gamepad1.right_stick_x);

        if(gamepad1.left_stick_y > 0.3 && distance.getDistance(DistanceUnit.INCH) < 6){
            frontLeft.setPower(.3);
            frontRight.setPower(.3);
        }
    }
    private void intake(){
        if (gamepad1.left_bumper) {
            claw.setPosition(0.1);
        }
        else if (gamepad1.right_bumper){
            claw.setPosition(0.9);
        }

        if (gamepad1.dpad_left){
            rotation.setPosition(.1);
        }
        else if(gamepad1.dpad_right){
            rotation.setPosition(.9);
        }

        if(gamepad1.a){
            arm.setPosition(0.1);
        }
        else if(gamepad1.b) {
            arm.setPosition(.9);
        }
        if(gamepad1.x){
            slide.setPower(1);
        }
        else if (gamepad1.y){
            slide.setPower(-1);
        }
        else{
            slide.setPower(0);
        }
        /*
        if(gamepad1.x){
            tower.setPosition(.1);
        }
        else if(gamepad1.y){
            tower.setPosition(.9);
        }
        */

    }

}
