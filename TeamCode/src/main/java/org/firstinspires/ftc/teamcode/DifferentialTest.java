package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "differential test")
public class DifferentialTest extends OpMode {

    private Servo left;
    private Servo right;

    private double diffValue;

    private boolean isLifted = false;



    @Override
    public void init() {
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");

        left.setDirection(Servo.Direction.REVERSE);


    }

    //left is in slot 4
    //right is in slot 3

    /*


    left serovo all pos fine for now
    right servo
     */

    @Override
    public void loop() {
        //handleDifferential();
        if (gamepad1.aWasPressed()) {

            left.setPosition(0.8);
            right.setPosition(0.8);
        } else if (gamepad1.yWasPressed()) {
            left.setPosition(0);
            right.setPosition(0);
        } else if (gamepad1.x) {

                left.setPosition(left.getPosition()+0.2);
                right.setPosition(right.getPosition()-0.2);

        } else if (gamepad1.b) {
            // right rotation
                left.setPosition(left.getPosition()-0.2);
                right.setPosition(left.getPosition()+0.2);

        }
    }

    private void handleDifferential() {
        if (gamepad1.aWasPressed()) {
            // lift it up
            isLifted = true;
            left.setPosition(1);
            right.setPosition(1);
        } else if (gamepad1.yWasPressed()) {
            // lower it down
            isLifted = false;
            left.setPosition(0);
            right.setPosition(0);
        } else if (gamepad1.xWasPressed()) {
            // left rotation
            if (isLifted) {
                left.setPosition(0.7);
                right.setPosition(0.0);
            } else {
                left.setPosition(0.0);
                right.setPosition(0.7);
            }
        } else if (gamepad1.bWasPressed()) {
            // right rotation
            if (isLifted) {
                left.setPosition(0.7);
                right.setPosition(0.7);
            } else {
                left.setPosition(0.0);
                right.setPosition(0.0);
            }
        }
    }
}
