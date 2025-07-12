package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "basic teleop")
public class eBasicTeleop extends OpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Servo claw;
    private Servo rotation;
    private Servo arm;
    private Servo tower;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        claw = hardwareMap.get(Servo.class, "claw");
        rotation = hardwareMap.get(Servo.class, "rotation");
        arm = hardwareMap.get(Servo.class, "arm");
        tower = hardwareMap.get(Servo.class, "tower");
    }

    @Override
    public void loop() {
        drivetrain();
        intake();

        telemetry.addData("frontLeft position", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight position", frontRight.getCurrentPosition());
        telemetry.addData("backLeft position", backLeft.getCurrentPosition());
        telemetry.addData("frontRight position", frontRight.getCurrentPosition());
    }
    private void drivetrain(){
        backLeft.setPower(1*gamepad1.left_stick_x);
        backRight.setPower(-1*gamepad1.left_stick_x);
        frontLeft.setPower(-1*gamepad1.left_stick_x);
        frontRight.setPower(1*gamepad1.left_stick_x);

        backLeft.setPower(1*gamepad1.left_stick_y);
        backRight.setPower(1*gamepad1.left_stick_y);
        frontLeft.setPower(1*gamepad1.left_stick_y);
        frontRight.setPower(1*gamepad1.left_stick_y);

        backLeft.setPower(-1 * gamepad1.right_stick_x);
        backRight.setPower(1*gamepad1.right_stick_x);
        frontLeft.setPower(-1*gamepad1.right_stick_x);
        frontRight.setPower(1*gamepad1.right_stick_x);
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
            tower.setPosition(.1);
        }
        else if(gamepad1.y){
            tower.setPosition(.9);
        }
    }

}
