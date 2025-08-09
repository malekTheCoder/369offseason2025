package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@TeleOp(name = "basic teleop")
public class eBasicTeleop extends OpMode {
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
    private NormalizedColorSensor color;
    double hue;
    double redVal;
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
        //color
        color = hardwareMap.get(NormalizedColorSensor.class, "color");

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

        telemetry.addData("frontLeft position", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight position", frontRight.getCurrentPosition());
        telemetry.addData("backLeft position", backLeft.getCurrentPosition());
        telemetry.addData("backRight position", backRight.getCurrentPosition());

        telemetry.addData("Light Detected", ((OpticalDistanceSensor) color).getLightDetected());
        NormalizedRGBA colors = color.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor());
        redVal = colors.red;

        if(redVal>0.08){
            claw.setPosition(0.9);
        }/*
        else{
            claw.setPosition(0.1)
        }*/


        //Determining the amount of red, green, and blue
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);

        //Determining HSV and alpha
        telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
        telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
        telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));
        telemetry.addData("Alpha", "%.3f", colors.alpha);

        //Using hue to detect color
        if(hue < 30){
            telemetry.addData("Color", "Red");
        }
        else if (hue < 60) {
            telemetry.addData("Color", "Orange");
        }
        else if (hue < 90){
            telemetry.addData("Color", "Yellow");
        }
        else if (hue < 150){
            telemetry.addData("Color", "Green");
        }
        else if (hue < 225){
            telemetry.addData("Color", "Blue");
        }
        else if (hue < 350){
            telemetry.addData("Color", "Purple");
        }
        else{
            telemetry.addData("Color", "Red");
            //claw.setPosition(0.9);
        }

        //telemetry.addData("slide position", slide.getCurrentPosition());
        telemetry.update();
    }
    private void drivetrain(){

        if(gamepad1.left_stick_x > 0.2) {
            backLeft.setPower(-1 * gamepad1.left_stick_x);
            backRight.setPower(1 * gamepad1.left_stick_x);
            frontLeft.setPower(1 * gamepad1.left_stick_x);
            frontRight.setPower(-1 * gamepad1.left_stick_x);
        }
        if(gamepad1.left_stick_x < -0.2){
            backLeft.setPower(-1 * gamepad1.left_stick_x);
            backRight.setPower(1 * gamepad1.left_stick_x);
            frontLeft.setPower(1 * gamepad1.left_stick_x);
            frontRight.setPower(-1 * gamepad1.left_stick_x);
        }

        if(gamepad1.left_stick_y > .2){
            backLeft.setPower(-gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.left_stick_y);
            frontLeft.setPower(-gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.left_stick_y);
        }
        if(gamepad1.left_stick_y < -0.2){
            backLeft.setPower(-gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.left_stick_y);
            frontLeft.setPower(-gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.left_stick_y);
        }

        if(gamepad1.right_stick_x > 0.2){
            backLeft.setPower(1 * gamepad1.right_stick_x);
            backRight.setPower(-1*gamepad1.right_stick_x);
            frontLeft.setPower(1*gamepad1.right_stick_x);
            frontRight.setPower(-1*gamepad1.right_stick_x);
        }

        if(gamepad1.right_stick_x <-0.2){
            backLeft.setPower(1 * gamepad1.right_stick_x);
            backRight.setPower(-1*gamepad1.right_stick_x);
            frontLeft.setPower(1*gamepad1.right_stick_x);
            frontRight.setPower(-1*gamepad1.right_stick_x);
        }

        if(gamepad1.left_stick_x==0 && gamepad1.right_stick_x==0 && gamepad1.left_stick_y==0){
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        if(gamepad1.left_stick_y > 0.3 && distance.getDistance(DistanceUnit.INCH) < 10){
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        if(distance.getDistance(DistanceUnit.INCH) < 20){
            arm.setPosition(0.1);
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
