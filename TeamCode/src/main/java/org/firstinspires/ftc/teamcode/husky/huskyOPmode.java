





package org.firstinspires.ftc.teamcode.husky;
import java.util.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="HuskyLens TeleOp", group="Vision")
public class huskyOPmode extends LinearOpMode {
    huskypipeline pipeline;
    HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        // Initialize the HuskyLens sensor from hardware map
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        // Set the HuskyLens to color detection mode (COLOR_RECOGNITION)
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        // Create and set our pipeline
        pipeline = new huskypipeline(huskyLens);



        // Wait for the start command
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Initialized. Press Play to start");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                pipeline.processDetections();
                List<huskypipeline.DetectedPose> detections = pipeline.getLatestDetections();

                telemetry.addData("Detected Objects", detections.size());
                int blockCounter = 0;

                for (huskypipeline.DetectedPose pose : detections) {
                    blockCounter++;
                    telemetry.addLine(String.format("Object " + blockCounter));
                    telemetry.addData(" Center (X,Y,Z) mm",
                            String.format("%.1f, %.1f, %.1f", pose.x, pose.y, pose.z));
                    telemetry.addData(" Distance (mm)", pose.distance);
                    telemetry.addData(" Yaw/Pitch/Roll (deg)",
                            String.format("%.1f / %.1f / %.1f", pose.yaw, pose.pitch, pose.roll));
                }


                telemetry.update();
                sleep(20);  // small delay to avoid spamming the loop
            }
        }

    }
}
