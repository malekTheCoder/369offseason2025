package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "test distance sensor malek")
public class DistanceSensorTest extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DistanceSensor dist

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
        double rStickY = -gamepad1.right_stick_y;
        if ((rStickY > 0.2) && (dist.getDistance(DistanceUnit.INCH) > 20)){
            frontLeft.setPower(rStickY);
            frontRight.setPower(rStickY);
            backLeft.setPower(rStickY);
            backRight.setPower(rStickY);

        } else if (rStickY < -0.2) {
            frontLeft.setPower(rStickY);
            frontRight.setPower(rStickY);
            backLeft.setPower(rStickY);
            backRight.setPower(rStickY);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
}
