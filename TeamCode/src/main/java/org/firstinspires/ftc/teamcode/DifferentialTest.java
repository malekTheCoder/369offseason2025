package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "differential test")
public class DifferentialTest extends OpMode {

    private Servo left;
    private Servo right;
    private Servo claw;


    @Override
    public void init() {
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        claw = hardwareMap.get(Servo.class, "claw");


    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left){

        }
        else if (gamepad1.dpad_right){

        }
        else if (gamepad1.dpad_up){

        }
        else if (gamepad1.dpad_down){

        }


    }
}
