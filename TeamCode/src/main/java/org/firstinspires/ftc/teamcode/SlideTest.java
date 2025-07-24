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
@TeleOp(name = "basic linear slide")
public class SlideTest extends OpMode {
    DcMotorEx linearSlide;


    @Override
    public void init() {
        linearSlide = hardwareMap.get(DcMotorEx.class, "frontLeft");

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setTargetPositionTolerance(50);



    }
//1780 max
    @Override
    public void loop() {

      // linearSlide.setPower(-gamepad1.left_stick_y*0.5);
       if((gamepad1.y) && (!(linearSlide.getCurrentPosition() > 1650))){
           linearSlide.setTargetPosition(1700);
           linearSlide.setPower(0.5);

       } else if (gamepad1.a && !(linearSlide.getCurrentPosition() < 100)) {
           linearSlide.setTargetPosition(50);
           linearSlide.setPower(-0.5);
       } else {
           linearSlide.setPower(0);
       }
        telemetry.addData("slide pos", linearSlide.getCurrentPosition());
       telemetry.update();
    }
}
