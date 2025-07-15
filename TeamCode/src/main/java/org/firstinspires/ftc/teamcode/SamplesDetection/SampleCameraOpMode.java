/*

notes
-> figure out what all the types of errorCodes are and what they mean
-> create a new opMode for inverse kinematics 3 axis arm


*/


package org.firstinspires.ftc.teamcode.SamplesDetection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SamplesDetection.SampleGamePiecePipeline;
import org.openftc.easyopencv.*;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class SampleCameraOpMode extends LinearOpMode {
    private OpenCvCamera camera;
    private SampleGamePiecePipeline pipeline;


    @Override
    public void runOpMode() {
        // Initialize camera using EasyOpenCV with gobilda calibration
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam"));
        pipeline = new SampleGamePiecePipeline();
        camera.setPipeline(pipeline);


        // Open camera at 1920x1080 resolution, 30 fps
        //camera.openCameraDeviceAsync(...); -> the method opens the camera and takes
        //an AsyncCameraOpenListener, letting you to run code when camera is opened or what to do when it fails
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            //new OpenCvCamera.AsyncCameraOpenListener() is an implementation of AsyncCameraOpenListener
            //there are only 2 methods -> onOpened(), onError()
            @Override
            public void onOpened() {
                //if Camera is opened and ready it starts streaming at specified resolution
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);//might have to change camera rotation
            }
            @Override
            public void onError(int errorCode) {
                //errorCode parameter is an integer value indicating the specific reason why the camera failed to open
                telemetry.addData("Camera error", errorCode);
            }
        });


        //telemetry for determining if camera is initialized and ready
        telemetry.addData("Status", "Initialized, waiting for start...");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            if (pipeline.isSampleDetected()) {
                //telemetry
                telemetry.addData("Sample Detected", "YES");
                telemetry.addData("Sample Center (px)", pipeline.sampleCenter);
                telemetry.addData("Sample Distance X (mm)", pipeline.posX);
                telemetry.addData("Sample Distance Y (mm)", pipeline.posY);
                telemetry.addData("Sample Distance Z (mm)", pipeline.posZ);
                telemetry.addData("Sample Yaw (deg)", pipeline.yaw);
            } else {
                telemetry.addData("Sample Detected", "NO");
            }
            telemetry.update();
            sleep(50);
        }
    }
}




