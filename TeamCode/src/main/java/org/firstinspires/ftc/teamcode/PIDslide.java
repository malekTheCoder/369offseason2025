package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import android.util.Range;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;




@Config
@TeleOp(name = "pid linear slide")
public class PIDslide extends OpMode {
    DcMotorEx linearSlide;
    PIDEx slidePID;

    public static PIDCoefficientsEx coeffs;

    double target = 0;


    @Override
    public void init() {
        linearSlide = hardwareMap.get(DcMotorEx.class, "slide");

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double Kp = 0.01;
        double Ki = 0;
        double Kd = 0;
        double integralCap = 300;
        double stabilityThreshold = 20;
        double lowPassAlpha = 0.85;

        coeffs = new PIDCoefficientsEx(Kp, Ki, Kd, integralCap, stabilityThreshold, lowPassAlpha);
        slidePID = new PIDEx(coeffs);
        //slidePID = new PIDEx(new PIDCoefficientsEx(Kp, Ki, Kd, integralCap, stabilityThreshold, lowPassAlpha));

    }

    @Override
    public void loop() {

        if (gamepad1.a) target = 0;
        if (gamepad1.b) target = 500;
        if (gamepad1.y) target = 1000;

        double current = linearSlide.getCurrentPosition();
        double output = slidePID.calculate(target, current);

        if (output > 1.0) {
            output = 1.0;
        } else if (output < -1.0) {
            output = -1.0;
        }

        linearSlide.setPower(output);

        telemetry.addData("Target -> ", target);
        telemetry.addData("Current -> ", current);
        telemetry.addData("Output", output);
        telemetry.update();

    }
}
