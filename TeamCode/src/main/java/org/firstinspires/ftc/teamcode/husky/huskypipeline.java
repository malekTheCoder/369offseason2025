package org.firstinspires.ftc.teamcode.husky;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;


public class huskypipeline extends OpenCvPipeline{
    // Camera intrinsics (from calibration)
    private static final double FX = 230.0;
    private static final double FY = 230.0;
    private static final double CX = 160.0;
    private static final double CY = 120.0;
    private final Mat cameraMatrix;
    private final MatOfDouble distCoeffs;

    // sample game piece model points (corners of the piece in 3D, in mm)
    private static final double PIECE_LENGTH = 88.9;  // 3.5 inches in mm
    private static final double PIECE_WIDTH  = 38.1;  // 1.5 inches in mm
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
        cameraMatrix = Mat.eye(3, 3, CvType.CV_64FC1);
        cameraMatrix.put(0, 0, FX);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, CX);
        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, FY);
        cameraMatrix.put(1, 2, CY);
        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);
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

    @Override
    public Mat processFrame(Mat input) {
        // Get all detected color blocks from HuskyLens
        HuskyLens.Block[] blocks = husky.blocks();
        // Prepare a list to hold pose results for this frame
        List<DetectedPose> frameDetections = new ArrayList<>();

        if (blocks != null) {
            for (HuskyLens.Block block : blocks) {
                // Get 2D corners of the bounding box in the HuskyLens image
                int left   = block.left;
                int top    = block.top;
                int width  = block.width;
                int height = block.height;
                // Compute corner pixel coordinates
                Point topLeft = new Point(left,top);
                Point topRight = new Point(left + width,top);
                Point bottomRight = new Point(left + width,top + height);
                Point bottomLeft = new Point(left,top + height);
                MatOfPoint2f imagePoints = new MatOfPoint2f(topLeft, topRight, bottomRight, bottomLeft);

                // Solve PnP to get rvec (rotation) and tvec (translation)
                Mat rvec = new Mat();
                Mat tvec = new Mat();
                boolean solved = Calib3d.solvePnP(
                        pieceObjectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec,
                        false, Calib3d.SOLVEPNP_ITERATIVE);
                if (!solved) {
                    continue;  // skip if solvePnP failed for some reason
                }

                // Convert rotation vector to rotation matrix
                Mat R = new Mat();
                Calib3d.Rodrigues(rvec, R);

                // Extract translation components (camera coordinate system)
                double X = tvec.get(0,0)[0];
                double Y = tvec.get(1,0)[0];
                double Z = tvec.get(2,0)[0];

                // Compute orientation angles (yaw, pitch, roll)
                // Yaw: horizontal angle (bearing) in degrees
                double yaw = Math.toDegrees(Math.atan2(X, Z));
                // Pitch: vertical angle (elevation) in degrees (note camera Y is down, so use -Y for conventional up-positive angle)
                double pitch = Math.toDegrees(Math.atan2(-Y, Z));
                // Roll: rotation around camera Z-axis. We derive it from the rotation matrix.
                // Here, roll is the tilt of the object about the line of sight.
                double roll;
                {
                    // Using the rotation matrix, compute roll as the rotation of the object’s X-axis relative to horizontal
                    // We assume camera is upright. Roll angle can be derived from the first two columns of R.
                    double r00 = R.get(0,0)[0], r01 = R.get(0,1)[0];
                    double r10 = R.get(1,0)[0], r11 = R.get(1,1)[0];
                    // roll = arctan2(R10, R00) in degrees (this assumes small tilt; alternative extraction can be used)
                    roll = Math.toDegrees(Math.atan2(r10, r11));
                }

                // Compute distance from camera to object center
                double distance = Math.sqrt(X*X + Y*Y + Z*Z);

                // Save the detection info
                DetectedPose pose = new DetectedPose();
                pose.x = X;
                pose.y = Y;
                pose.z = Z;
                pose.distance = distance;
                pose.yaw = yaw;
                pose.pitch = pitch;
                pose.roll = roll;
                frameDetections.add(pose);

                // Optionally, draw the bounding box and axes on the image for debugging
                Imgproc.rectangle(input, topLeft, bottomRight, new Scalar(0, 255, 0), 2);
                // Draw a small cross at the center of the bounding box
                int centerX = block.x;
                int centerY = block.y;
                Imgproc.drawMarker(input, new Point(centerX, centerY), new Scalar(0,255,0), Imgproc.MARKER_CROSS, 10, 2);
                // (For a more advanced visualization, one could project a 3D axis using Calib3d.projectPoints and draw it)
            }
        }

        // Update the latestDetections list (synchronized for thread safety)
        synchronized (this) {
            latestDetections.clear();
            latestDetections.addAll(frameDetections);
        }

        // Return the annotated frame to display on the Driver Station (if using camera preview)
        return input;
    }
}
