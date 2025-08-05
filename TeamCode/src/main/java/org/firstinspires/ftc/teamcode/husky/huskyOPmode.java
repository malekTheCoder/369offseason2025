package org.firstinspires.ftc.teamcode.husky;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="HuskyLens TeleOp", group="Vision")
public class huskyOPmode extends LinearOpMode {
    OpenCvCamera camera;
    huskypipeline pipeline;
    HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        // Initialize the HuskyLens sensor from hardware map
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        // Set the HuskyLens to color detection mode (COLOR_RECOGNITION)
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // Initialize the OpenCV camera (assume a webcam is used)
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Create and set our pipeline
        pipeline = new huskypipeline(huskyLens);
        camera.setPipeline(pipeline);

        // Open camera device and start streaming images to the pipeline
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera failed to open!");
            }
        });

        // Wait for the start command
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Initialized. Press Play to start");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get latest detections from the pipeline
                java.util.List<huskypipeline.DetectedPose> detections = pipeline.getLatestDetections();
                telemetry.addData("Detected Objects", detections.size());
                int i = 0;
                for (huskypipeline.DetectedPose pose : detections) {
                    i++;
                    // Telemetry for each object: position, distance, angles
                    telemetry.addLine(String.format("Object" + i));
                    telemetry.addData(" Center (X,Y,Z) mm",
                            String.format("%.1f, %.1f, %.1f", pose.x, pose.y, pose.z));
                    telemetry.addData(" Distance (mm) ", pose.distance);
                    telemetry.addData(" Yaw/Pitch/Roll (deg)",
                            String.format("%.1f / %.1f / %.1f", pose.yaw, pose.pitch, pose.roll));
                }
                telemetry.update();
                sleep(20);  // small delay to avoid spamming the loop
            }
        }

        // Stop streaming when op mode ends
        if (camera != null) {
            camera.stopStreaming();
        }
    }
}
