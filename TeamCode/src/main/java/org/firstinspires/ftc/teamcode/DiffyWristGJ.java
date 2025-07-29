package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

/*
notes for diffy wrist and explaining the math

pitch is for up and down -> if you want to move the wrist up/down
you move both servos up/down by the same amount

roll is for right and left twist
-> to twist left, one servo goes up and the other goes down
-> vice-versa for right twist.

the math formulat is the following
-> left servo is theta 1
-> right servo is theta 2

the pitch angle is theta p
the roll angle is theta r

pitch and roll is caculated for each servo:

theta 1 = theta p + (theta r/2)
theta 2 = theta p - (theta r/2)

-> for pitch add the same amount to both
-> for roll add half to one, subtract half from the other -> effectively making the total
change theta r



 */

@TeleOp (name = "diffyWristGJ")
public class DiffyWristGJ extends OpMode
{
    private Servo left;
    private Servo right;

    private static final double MAX_SERVO_RANGE = 300.0;
    private static final double MAX_SERVO_ANGLE = 300.0;
    private static final double MIN_SERVO_ANGLE = 0.0;

    @Override
    public void init(){
        left = hardwareMap.get(Servo.class, "leftServo");
        right = hardwareMap.get(Servo.class, "rightServo");

    }

    @Override
    public void loop(){
        //this teleop will use the joy sticks to controll the pitch and roll
        double pitchInput = -gamepad1.left_stick_y; //up/down
        double rollInput = gamepad1.left_stick_x; //left/right turn


        //given that the joysticks go from -1 to +1
        //we have to multiply by the maximum angle to turn the joystick
        //movement into a real angle
        //and example is -> joystick halfway up -> (0.5)
        //max pitch rang is 90 degrees, desired pitch would be -> 0.5 x 90 = 45 degrees
        double maxPitchRange = 90;
        double maxRollRange = 60;

        double desiredPitch = pitchInput * maxPitchRange;
        double desiredRoll = rollInput * maxRollRange;



        //now we can calculate the servo angels
        //the 150 is used as a neutral position or the upright position
        //-> need to work with kaichen to determine the neutral position
        //the rest of the formula is the same as before
        double theta1 = 150+desiredPitch + (desiredRoll/2); // -> assuming the servo is centered at 150
        double theta2 = 150+desiredPitch - (desiredRoll/2);

        //now setting servo limits
        //this is just used to make sure the numbers for each servo
        //angle stay within 0 to 300 degrees so we don't break anything
        theta1 = Math.max(MIN_SERVO_ANGLE, Math.min(MAX_SERVO_ANGLE, theta1));
        theta2 = Math.max(MIN_SERVO_ANGLE, Math.min(MAX_SERVO_ANGLE, theta2));


        //converting from angles to positions that the servo can read
        double pos1 = theta1 / MAX_SERVO_RANGE;
        double pos2 = theta2 / MAX_SERVO_RANGE;

        //sertting servos at intended positions
        left.setPosition(pos1);
        right.setPosition(pos2);

        //telemetry
        telemetry.addData("PitchInput", desiredPitch);
        telemetry.addData("Roll Input", desiredRoll);
        telemetry.addData("Servo1 Angle", theta1);
        telemetry.addData("Servo2 Angle", theta2);
        telemetry.addData("Servo1 Pos", pos1);
        telemetry.addData("Servo2 Pos", pos2);
        telemetry.update();

    }


}
