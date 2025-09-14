package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
the plan for pipelines is:

**we need one pipeline for every tag

Pipeline 0 → GOAL Blue (ID 20), Full 3D targeting, 36h11, marker size 20.65 cm, ID filter = 20.

Pipeline 1 → GOAL Red (ID 24), same settings, ID filter = 24.

Pipelines 2–4 → OBELISK IDs 21/22/23 to read the motif; don’t use their pose for localization.

**future iterations of this code will add pipeline switching
**right now just have to get one pipeline set on ui 

**additionally the offsets for xyz and roll pitch yaw can be set on the web ui for limelight

 */



@TeleOp(name = "Limelight localization")
public class LimeLightLocalizaiton extends LinearOpMode {

    //choose pipeline
    private static final int DEFAULT_APRILTAG_PIPE = 0;

    //offsets
    // Convention: x = +forward, y = +RIGHT, z = +up  (z=0 at top of base; x=y=0 at robot center)
    private static final double CAM_X_M        = 0.0;   // +Forward
    private static final double CAM_Y_RIGHT_M  = 0.0;   // +RIGHT
    private static final double CAM_Z_M        = 0.0;   // +UP from base

    private static final double CAM_ROLL_BIAS_DEG  = 0.0;
    private static final double CAM_PITCH_BIAS_DEG = 0.0;
    private static final double CAM_YAW_BIAS_DEG   = 0.0;

    // If true, subtract camera->robot-center translation from the LimeLight field pose
    private static final boolean APPLY_SOFT_XY_OFFSETS = true;


    private static final String LIMELIGHT_NAME = "limelight";
    private static final String IMU_NAME       = "imu";

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usb  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
        imu.resetYaw();


        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(60);
        limelight.pipelineSwitch(DEFAULT_APRILTAG_PIPE);   //  pipeline chosen in code

        telemetry.setMsTransmissionInterval(20);
        telemetry.addLine("LL3A Localize (pipeline set in code)");
        telemetry.addData("Pipeline", DEFAULT_APRILTAG_PIPE);
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            // Push IMU yaw (deg, CCW+) for MegaTag2 fused pose (LimeLight will use it in MT2)
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            double imuYawDeg = ypr.getYaw(AngleUnit.DEGREES) + CAM_YAW_BIAS_DEG;
            limelight.updateRobotOrientation(imuYawDeg);

            // Read latest vision result
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Prefer fused pose (MT2) if available, else plain (MT1)
                Pose3D pose = (result.getBotpose_MT2() != null) ? result.getBotpose_MT2()
                        : result.getBotpose();

                if (pose != null) {
                    // Field pose (meters) as reported by LimeLight
                    double fieldX_m = pose.getPosition().x;
                    double fieldY_m = pose.getPosition().y;
                    double fieldZ_m = pose.getPosition().z;


                    YawPitchRollAngles rot = pose.getOrientation();
                    double roll_deg  = rot.getRoll(AngleUnit.DEGREES)  + CAM_ROLL_BIAS_DEG;
                    double pitch_deg = rot.getPitch(AngleUnit.DEGREES) + CAM_PITCH_BIAS_DEG;
                    double yaw_deg   = rot.getYaw(AngleUnit.DEGREES);

                    // software correction from camera origin to robot center (XY only)
                    if (APPLY_SOFT_XY_OFFSETS && (CAM_X_M != 0.0 || CAM_Y_RIGHT_M != 0.0)) {
                        double cos = Math.cos(Math.toRadians(yaw_deg));
                        double sin = Math.sin(Math.toRadians(yaw_deg));
                        // y is RIGHT-positive, so note the signs vs. a left-positive frame:
                        double dx_field =  CAM_X_M * cos + CAM_Y_RIGHT_M * sin;
                        double dy_field =  CAM_X_M * sin - CAM_Y_RIGHT_M * cos;

                        fieldX_m -= dx_field;
                        fieldY_m -= dy_field;
                    }

                    // Raw targeting  -> basically getting the angular offsets
                    // tx -> horizontal angular offset from the image center to target's targeting point
                    // ty -> vertical angular offset from image center to targeting point
                    // ta -> target area as a percentage of the image.
                    double tx = result.getTx();
                    double ty = result.getTy();
                    double ta = result.getTa();

                    telemetry.addData("Pipeline", DEFAULT_APRILTAG_PIPE);
                    telemetry.addLine("-- Pose (FIELD) --");
                    telemetry.addData("X (m): ",fieldX_m);
                    telemetry.addData("Y (m): ",fieldY_m);
                    telemetry.addData("Z (m): ",fieldZ_m);
                    telemetry.addData("Roll/Pitch/Yaw (deg)", "%.1f / %.1f / %.1f",
                            roll_deg, pitch_deg, yaw_deg);

                    telemetry.addLine("-- Targeting --");
                    telemetry.addData("tx (deg): ", tx);
                    telemetry.addData("ty (deg): ", ty);
                    telemetry.addData("ta (%): ", ta);
                } else {
                    telemetry.addData("Pipeline", DEFAULT_APRILTAG_PIPE);
                    telemetry.addLine("No pose this frame (not enough geometry).");
                }
            } else {
                telemetry.addData("Pipeline", DEFAULT_APRILTAG_PIPE);
                telemetry.addLine("No valid AprilTag target yet.");
            }

            telemetry.update();
        }
    }
}
