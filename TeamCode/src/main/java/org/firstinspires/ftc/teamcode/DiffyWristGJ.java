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

        double maxPitchRange = 90;
        double maxRollRange = 60;

        double desiredPitch = pitchInput * maxPitchRange;
        double desiredRoll = rollInput * maxRollRange;

        //now we can calculate the servo angels
        double theta1 = 150+desiredPitch + (desiredRoll/2); // -> assuming the servo is centered at 150
        double theta2 = 150+desiredPitch - (desiredRoll/2);

        //now setting servo limits
        theta1 = Math.max(MIN_SERVO_ANGLE, Math.min(MAX_SERVO_ANGLE, theta1));
        theta2 = Math.max(MIN_SERVO_ANGLE, Math.min(MAX_SERVO_ANGLE, theta2));


        double pos1 = theta1 / MAX_SERVO_RANGE;
        double pos2 = theta2 / MAX_SERVO_RANGE;

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
