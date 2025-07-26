package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@TeleOp(name = "basic teleop with pinpoint")
public class eTeleopPinpoint extends OpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    //private DcMotor slide;
    private Servo claw;
    private Servo rotation;
    private Servo arm;
    //private Servo tower;
    private DistanceSensor distance;

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        //TODO: declare offsets of the pinpoint device on the robot
        //pinpoint.setOffsets();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //TODO: Verify the directin of both encoders of odo pods and change below if needed
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //slide = hardwareMap.get(DcMotor.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        rotation = hardwareMap.get(Servo.class, "rotation");
        arm = hardwareMap.get(Servo.class, "sampleArm");
        //tower = hardwareMap.get(Servo.class, "tower");

        distance = hardwareMap.get(DistanceSensor.class, "distance");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.addLine("Hardware Initialized");
        telemetry.update();


        //slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        drivetrain();
        intake();

        if (gamepad1.x) {
            pinpoint.resetPosAndIMU();
        }

        telemetry.addData("frontLeft position", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight position", frontRight.getCurrentPosition());
        telemetry.addData("backLeft position", backLeft.getCurrentPosition());
        telemetry.addData("backRight position", backRight.getCurrentPosition());
        //telemetry.addData("slide position", slide.getCurrentPosition());
    }
    private void drivetrain(){

        pinpoint.update();



        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 0.7;


        double botHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        // Field-oriented adjustments
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        if(gamepad1.left_stick_y > 0.3 && distance.getDistance(DistanceUnit.INCH) < 10){
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        if(distance.getDistance(DistanceUnit.INCH) < 10){
            arm.setPosition(0.1);
        }

        Pose2D pos = pinpoint.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", pinpoint.getVelX(DistanceUnit.MM), pinpoint.getVelY(DistanceUnit.MM), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());

        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

        // telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        telemetry.update();



    }
    private void intake(){
        if (gamepad1.left_bumper) {
            claw.setPosition(0.1);
        }
        else if (gamepad1.right_bumper){
            claw.setPosition(0.9);
        }

        if (gamepad1.dpad_left){
            rotation.setPosition(.2);
        }
        else if(gamepad1.dpad_right){
            rotation.setPosition(.8);
        }

        if(gamepad1.a){
            arm.setPosition(0.1);
        }
        else if(gamepad1.b) {
            arm.setPosition(.9);
        }
/*
        if(gamepad1.x){
            slide.setPower(1);
        }
        else if (gamepad1.y){
            slide.setPower(-1);
        }
        else{
            slide.setPower(0);
        }*/
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
