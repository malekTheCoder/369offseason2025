package org.firstinspires.ftc.teamcode.SamplesDetection;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;


/*


   notes
   -> for detecting multiple samples & toggling which color to detect via gamepad


   we would have to detect all samples
   store all valid RotatedRect objects in a List, not just bestRect.




   -> for each detected contour that matches aspect ratio -> we would need to store its rectangle and center
   color toggling would mean setting which color bounds to use based on gamepad input


*/


public class SampleGamePiecePipeline extends OpenCvPipeline {
    // Camera calibration parameters for 1920x1080 (from goBILDA guide)
    public double fx = 1432.032, fy = 1432.032;  // Focal lengths in pixels
    public double cx = 997.085, cy = 1432.032;   // Principal point in pixels
    public double[] distCoeffs = {0.1289180, -0.3621222, 0.0, 0.2872672, 0.0, 0, 0, 0};


    // Real world sample dimensions (mm)
    public double SAMPLE_WIDTH = 50.8;   // 2.0 inches
    public double SAMPLE_LENGTH = 101.6; // 4.0 inches


    // Output variables for tuning/telemetry
    public Point sampleCenter = null;//set to null initially because we don't know if we've found any samples yet
    //if samples are found it is set to the center of a sample






    //these variables are are output variables for the samples position and orientation relative to camera
    public double posX = 0;
    public double posY = 0;
    public double posZ = 0;
    public double yaw = 0;
    public double pitch = 0;
    public double roll = 0;


    @Override
    public Mat processFrame(Mat input) {
        // 1. Convert to HSV for robust color filtering
        Mat hsv = new Mat();
        //imgproc is OpenCV's processing class
        //contains methods for converting images, drawing shapes, finding contours
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);


        // 2. Filter for yellow (tune for best results on your field lighting)
        Scalar lowerYellow = new Scalar(18, 80, 120);  // lower bound for yellow
        Scalar upperYellow = new Scalar(35, 255, 255); // upper bound for yellow


       /*
       * the following are scalars for red and blue samples
       *
           // For blue:
           Scalar lowerBlue = new Scalar(100, 100, 50);
           Scalar upperBlue = new Scalar(130, 255, 255);
           // For red (red is tricky because it wraps around in HSV)
           Scalar lowerRed1 = new Scalar(0, 100, 100);
           Scalar upperRed1 = new Scalar(10, 255, 255);
           Scalar lowerRed2 = new Scalar(160, 100, 100);
           Scalar upperRed2 = new Scalar(179, 255, 255);


       * */


        Mat mask = new Mat();
        //core in range checks every pixel in input (in this case the hsv)
        //to see if its between the lower and upper bounds
        //if in range = white otherwise = black
        //this will create the mask
        Core.inRange(hsv, lowerYellow, upperYellow, mask);


        // 3. Find contours in mask
        //Contour is a curve joining all the continuous points along the boundary
        //having the same color/intensity


        //the array list is made because imgproc.findContours fills the list with all contours it finds
        //each entry in the array list is one detected contour/shape


        //MatOfPoint is an OpenCV class representing a list of 2D points
        //a contour is a set of (x,y) points around the shape's edge
        List<MatOfPoint> contours = new ArrayList<>();
        //hierarchy Mat holds information about how contours are nested
        //nested basically means one shape inside another
        //we ignore this for finding simple objects but good to know about
        Mat hierarchy = new Mat();
       /*Imgproc.findContours(...);
       mask: The black-and-white image to search for shapes.
       contours: The list we want to fill with found contours.
       hierarchy: Info about how contours are nested; not needed here.
       Imgproc.RETR_EXTERNAL: Only find the outermost contours (ignore nested holes).
       Imgproc.CHAIN_APPROX_SIMPLE: Compresses horizontal, vertical, and diagonal segments, saving memory.
       * */
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);




        //maxArea helps keep track of the biggest detected contour
        //assuming that the largest sample in the frame is going to be the closest
        //it is set to 0 because no contours have been checked
        double maxArea = 0;
        //RotatedRect holds a rectangle that can be at any rotation angle
        //bestRect will be set to largest, most rectangle-like yellow blob
        RotatedRect bestRect = null;//again set to null becuase we haven't found any yet








        // 4. Loop over all contours to find the largest yellow rectangular contour
        for (MatOfPoint cnt : contours) {
            double area = Imgproc.contourArea(cnt);
            //area will stand for the number of pixels inside the contour (cnt)
            //basically it will determine how big the blob is




            if (area > 500) { // Ignore small areas (noise)
                //rect represetnst the smallest rectangle at any angle that can
                //wrap around the contour at hand
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(cnt.toArray()));


                //width and ratio are used to determine whether the contour is an actual sample
                double width = rect.size.width, height = rect.size.height;
                double ratio = Math.max(width, height) / Math.min(width, height);


                //now we are using width and ratio to determine if the contour is a sample
                if (ratio > 1.6 && ratio < 2.5 && area > maxArea) { // sample aspect is ~2:1
                    //we are only keeping rectangles that look like
                    //samples


                    //if the new contour/sample is bigger than the previous one we keep it
                    maxArea = area;
                    //bestRect hold the most rectangle-like yellow blob found
                    bestRect = rect;
                }
            }
        }






        // 5. If a valid sample is found, draw outline, center dot, and estimate pose
        if (bestRect != null) {
            Point[] boxPts = new Point[4];// the list is made to hold the four corners of the rectangle
            bestRect.points(boxPts);//this will fill the boxPts array with four (x,y) positions of the rectanlge's corners


            //the for-loop will draw a green line between each corner of the rectangle
            for (int i = 0; i < 4; i++)
                Imgproc.line(input, boxPts[i], boxPts[(i+1)%4], new Scalar(0,255,0), 4);




            //bestRect.center gives the (x,y) pixel locatino at the middle of the rectangle (center of rectangle)
            sampleCenter = bestRect.center;
            //imgproc.circle(...); only thickens the center dot
            Imgproc.circle(input, sampleCenter, 8, new Scalar(0,255,0), -1);


            // 6. Calculate 3D position and orientation using solvePnP


            //solvePnP figures out where the object is in 3D space and how its roatated realtive to the camera
            //it stands for Solve Perspective-n-Point
            //solvePnP takes 3D coords of points on real object along with 2D coords of points in image
            //it solves for how the object is rotated in 3D relative to the camera and where the object is located relative to the camera
            MatOfPoint3f objectPts = new MatOfPoint3f(


                    //these will define where the 4 corners are in real life relative to sample's center
                    new Point3(-SAMPLE_LENGTH/2, -SAMPLE_WIDTH/2, 0),
                    new Point3(SAMPLE_LENGTH/2, -SAMPLE_WIDTH/2, 0),
                    new Point3(SAMPLE_LENGTH/2, SAMPLE_WIDTH/2, 0),
                    new Point3(-SAMPLE_LENGTH/2, SAMPLE_WIDTH/2, 0)
            );






            MatOfPoint2f imagePts = new MatOfPoint2f(boxPts);




            //A camera matrix is a 3x4 matrix that describes how a 3D world point is projected onto a 2D image plane.
            Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);


            //the first 0,0 parameters indicates OpenCV to start at the 0,0 position of the 3x3 matrix
            //the other 9 params are values for the matrix
            cameraMatrix.put(0,0,
                    fx, 0, cx,
                    0, fy, cy,
                    0, 0, 1);


            //distortion matrix holds numbers that tell OpenCV how the specific camera bends light (the coefficients found in gobilda user setup guide)
            MatOfDouble dist = new MatOfDouble(distCoeffs);


            //the rotation vector and translation vectors are outputs from solvePnP
            //rvec describes how the object is rotated
            Mat rvec = new Mat();
            //tvec describes where the object is located in space (xyz)
            Mat tvec = new Mat();




           /*
           just for personal note I have not seen the try/catch stuff before and took a while to stumble across this
           if something inside the try block causes an error the code catches the block instead of crashing


           it comes in handy as it protects against the rare case where solvePnP fails due to invalid points
            */
            boolean pnpOk = false;
            try {
                pnpOk = Calib3d.solvePnP(objectPts, imagePts, cameraMatrix, dist, rvec, tvec);
            } catch (Exception e) {
                // solvePnP may throw if points are not valid
            }


            if (pnpOk) {
                posX = tvec.get(0,0)[0];//first element of xyz translations
                posY = tvec.get(1,0)[0];
                posZ = tvec.get(2,0)[0];


                // Convert rotation vector to rotation matrix
                //rvec is an axis-angle representation
                //getting standard rotation would use Rodrigues formula
                Mat rotMat = new Mat();
                Calib3d.Rodrigues(rvec, rotMat);


                // Extract yaw, pitch, roll from rotation matrix (in degrees)
                // Assumes OpenCV convention: [R] = [Rz(yaw)][Ry(pitch)][Rx(roll)]
                // See: https://www.learnopencv.com/rotation-matrix-to-euler-angles/
                //these are the values for the 3x3 matrix used to calculate the Euler angles
                double m00 = rotMat.get(0,0)[0];
                double m01 = rotMat.get(0,1)[0];
                double m02 = rotMat.get(0,2)[0];
                double m10 = rotMat.get(1,0)[0];
                double m11 = rotMat.get(1,1)[0];
                double m12 = rotMat.get(1,2)[0];
                double m20 = rotMat.get(2,0)[0];
                double m21 = rotMat.get(2,1)[0];
                double m22 = rotMat.get(2,2)[0];






               /*
               First the if statement checks if the rotation matix is in a gimbal lock where the orientation is
               such that you cannot uniquely determine all three angles


               if there is no gimbal lock we use trigonometry to find pitch roll yaw from matrix elements
               if there is a gimbal lock, other formulas are used
                */
                // Calculate Euler angles (yaw, pitch, roll)
                if (Math.abs(m20) != 1) {
                    pitch = -Math.asin(m20);
                    roll = Math.atan2(m21 / Math.cos(pitch), m22 / Math.cos(pitch));
                    yaw = Math.atan2(m10 / Math.cos(pitch), m00 / Math.cos(pitch));
                } else {
                    // Gimbal lock case
                    yaw = 0;
                    if (m20 == -1) {
                        pitch = Math.PI / 2;
                        roll = yaw + Math.atan2(m01, m02);
                    } else {
                        pitch = -Math.PI / 2;
                        roll = -yaw + Math.atan2(-m01, -m02);
                    }
                }
                // Convert radians to degrees
                yaw = Math.toDegrees(yaw);
                pitch = Math.toDegrees(pitch);
                roll = Math.toDegrees(roll);
            }
        } else {
            sampleCenter = null;
            posX = posY = posZ = yaw = pitch = roll = 0;
        }


        return input;
    }


    public boolean isSampleDetected() {
        return sampleCenter != null;
    }
}
