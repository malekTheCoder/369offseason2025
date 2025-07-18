package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorTest extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DistanceSensor dist;


    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        dist = hardwareMap.get(DistanceSensor.class, "distance");

    }

    @Override
    public void loop() {
        handleDrivtrain();

        telemetry.addData("distance sensor", dist.getDistance(DistanceUnit.INCH));

    }

    private void handleDrivtrain() {
        double lStickY = gamepad1.left_stick_y;
        if ((lStickY > Math.abs(0.2)) && (dist.getDistance(DistanceUnit.INCH) > 12)){
            frontLeft.setPower(0.5 * lStickY);
            frontRight.setPower(0.5 * lStickY);
            backLeft.setPower(0.5 * lStickY);
            backRight.setPower(0.5 * lStickY);

        }
        else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
}
