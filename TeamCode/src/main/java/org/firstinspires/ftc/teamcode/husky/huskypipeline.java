package org.firstinspires.ftc.teamcode.husky;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;


public class huskypipeline{
    // Camera intrinsics (from calibration)
    private static final double FX = 230.0;
    private static final double FY = 230.0;
    private static final double CX = 160.0;
    private static final double CY = 120.0;
    private final Mat cameraMatrix;
    private final MatOfDouble distCoeffs;

    // sample game piece model points (corners of the piece in 3D, in mm)
    private static final double PIECE_LENGTH = 49.9;  // 3.5 inches in mm for actual sample
    private static final double PIECE_WIDTH  = 31.8;  // 1.5 inches in mm for actual sample
    private final MatOfPoint3f pieceObjectPoints;

    // Pose result container for each detection
    public static class DetectedPose {
        public double x;
        public double y;
        public double z;// position (mm)
        public double distance;// distance from camera (mm)
        public double yaw;
        public double pitch;
        public double roll; // orientation (deg)
    }
    private final List<DetectedPose> latestDetections = new ArrayList<>();

    // Reference to the HuskyLens sensor
    private HuskyLens husky;
    public huskypipeline(HuskyLens huskyLens) {
        this.husky = huskyLens;
        // Initialize camera matrix
        cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
        cameraMatrix.put(0, 0, FX, 0, CX,
                0, FY, CY,
                0, 0, 1);
        distCoeffs = new MatOfDouble(0,0,0,0);  // no distortion

        // Define 3D object corner points for the piece
        Point3[] corners = new Point3[] {
                new Point3(-PIECE_LENGTH/2.0,  PIECE_WIDTH/2.0, 0),  // top-left
                new Point3( PIECE_LENGTH/2.0,  PIECE_WIDTH/2.0, 0),  // top-right
                new Point3( PIECE_LENGTH/2.0, -PIECE_WIDTH/2.0, 0),  // bottom-right
                new Point3(-PIECE_LENGTH/2.0, -PIECE_WIDTH/2.0, 0)   // bottom-left
        };
        pieceObjectPoints = new MatOfPoint3f(corners);
    }

    /** Returns the latest list of detected poses (from the last frame). */
    public synchronized List<DetectedPose> getLatestDetections() {
        // Return a copy to avoid concurrency issues
        return new ArrayList<>(latestDetections);
    }


    public void processDetections() {
        //getting all detected color blocks from HuskyLens
        HuskyLens.Block[] blocks = husky.blocks();
        //prepare a list to hold a pose results for this frame
        List<DetectedPose> frameDetections = new ArrayList<>();

        if (blocks != null) {
            for (HuskyLens.Block block : blocks) {
                //getting the 2D corners for the bounding box for the solvePnP
                int left   = block.left;
                int top    = block.top;
                int width  = block.width;
                int height = block.height;
                //computing the pixel cordinates for the corners
                Point topLeft = new Point(left, top);
                Point topRight = new Point(left + width, top);
                Point bottomRight = new Point(left + width, top + height);
                Point bottomLeft = new Point(left, top + height);
                MatOfPoint2f imagePoints = new MatOfPoint2f(topLeft, topRight, bottomRight, bottomLeft);
                //solve pnp
                Mat rvec = new Mat();
                Mat tvec = new Mat();
                boolean solved = Calib3d.solvePnP(
                        pieceObjectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec,
                        false, Calib3d.SOLVEPNP_ITERATIVE);
                if (!solved) continue;
                //converting the rotation vector to rotation matrix
                Mat R = new Mat();
                Calib3d.Rodrigues(rvec, R);
                //extracting the translation components
                double X = tvec.get(0,0)[0];
                double Y = tvec.get(1,0)[0];
                double Z = tvec.get(2,0)[0];

                //computing orientation
                //Yaw -> horizontal angle
                //pitch -> vertical angle (elevation)
                //roll -> rotation around camera z-axis -> derived from rotation matrix
                double yaw = Math.toDegrees(Math.atan2(X, Z));
                double pitch = Math.toDegrees(Math.atan2(-Y, Z));

                //simple terms for r10 and r11 ->
                //r10 -> tells you how much X (right) direction points into cameras vertical axis
                //r11 -> tells you how much X points into the cameras horizontal axis
                //ratio gives you rotation of the object around Z-axis
                double r10 = R.get(1,0)[0], r11 = R.get(1,1)[0];
                double roll = Math.toDegrees(Math.atan2(r10, r11));

                double distance = Math.sqrt(X*X + Y*Y + Z*Z);

                //save the detection info
                DetectedPose pose = new DetectedPose();
                pose.x = X;
                pose.y = Y;
                pose.z = Z;
                pose.distance = distance;
                pose.yaw = yaw;
                pose.pitch = pitch;
                pose.roll = roll;
                frameDetections.add(pose);
            }
        }
        //update the latestDetections list (synchronized for thread safety
        synchronized (this) {
            latestDetections.clear();
            latestDetections.addAll(frameDetections);
        }
    }

}
