package frc.robot;


import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;


import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.*;
import org.opencv.core.Core.*;
//import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;


public class VisionHelper
{
    public static double[] centerCoor;
    public MatOfPoint undistort(Mat img, Mat mapx, Mat mapy) {
        //Mat img = Imgcodecs.imread(getClass().getResource("/fname.png").getPath()); // don't use this line, just use the inputted one
       // Mat gray = null;
        //Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);
        MatOfPoint dst = null;
        Imgproc.remap(img, dst, mapx, mapy, Imgproc.INTER_LINEAR); 
        return dst;
    }
    public VisionHelper() {
        
    }

    // returns -1, -1 if no contours found or multiple contours found
    public double[] findCenter(Mat img) {
        //[x,y]
        double[] centerCoor = {-1, -1}; // set in case of exception

        GripPipeline pipeline = new GripPipeline();
        pipeline.process(img);

        if(pipeline.filterContoursOutput().size() == 1){
            Moments moments = Imgproc.moments(pipeline.filterContoursOutput().get(0));
            centerCoor[0] = moments.get_m10() / moments.get_m00();
            centerCoor[1] = moments.get_m01() / moments.get_m00();
            return centerCoor;
        }
        return centerCoor;
    }
    
    public double convert_dist(double pixel_dist, double height){
        return 0.0001 * (9.081 * height * pixel_dist);
    }

    //returns slope of line, returns -1 if no contour found OR multiple contours found
    public double find_longer_line(Mat img){
        GripPipeline pipeline = new GripPipeline();
        pipeline.process(img);

        if(pipeline.filterContoursOutput().size() == 1){
            MatOfPoint contours = pipeline.filterContoursOutput().get(0);
            //returns m, y0, and x0 of longer line
            MatOfPoint2f myPt = new MatOfPoint2f();
            contours.convertTo(myPt, CvType.CV_32FC2);
            RotatedRect rc = Imgproc.minAreaRect(myPt);
            Mat box = new Mat();    
            Imgproc.boxPoints(rc, box); 

            ArrayList<Tup> tups = new ArrayList<Tup>(); //list of tuple

            // box.rows() should be the number of points, and each row is a point
            for (int i = 0; i < box.rows(); i++){
                for (int j = 0; j < box.rows(); j++){
                    if (i < j){
                        //double ydiff = box.get(j).get(1) - box.get(i).get(1);// difference in y coords
                        double[] y2 = box.get(j,1);
                        double[] y1 = box.get(i,1);
                        double ydiff = y2[0] - y1[0];// difference in y coords
                        //double xdiff = box.get(j).get(0) - box.get(i).get(0);; //difference in x coords
                        double[] x2 = box.get(j,0);
                        double[] x1 = box.get(i,0);
                        double xdiff = x2[0] - x1[0]; //difference in x coords
                        Double distance = Math.sqrt(xdiff * xdiff + ydiff * ydiff); //distance formula to find distance between 2 points
                        double slope = ydiff / xdiff;
                        Tup t = new Tup(distance, slope, box.row(i), box.row(j));
                        tups.add(t); //add in the tuple into the list 
                    }
                }
            }

            Collections.sort(tups);
            return tups.get(2).slope;
        }

        return -1;
    }

//########################################## 2.3b: ANGLE FROM TAPE SIDE TO CAMERA FACING #####################################################

    public double getCameraToTapeTheta(double m){
        //y = y0 + m(x - x0)
        //using one point x = x0, another point x = x0 + 100
        //find two points on the line. camera forward defines the y of the image, so finding the angle from the camera line is the same as finding the
        //angle from just the y axis. After two points on the line are found, find delta y and delta x, and then get atan.

        //ARC TAN OF THE SLOPE BASICALLY
        // but it's centered on degree pi/2 (if slope is infinity, i.e. straight up line, we want angle 0, not pi/2)
        // don't need to do pi/2 hack anymore
        return Math.atan(m);
        
    }

    // returns -1, -1 if no contour found postfilter
    // note that convention here is: negative theta is clockwise (x negative -> theta is also negative)
    // currently does not accept a robot_offset_x, we will need to calculate more (and do casework on weird
    // scenarios so it's significant complexity)
    // now we also don't accept tape_offset_y since we can't figure out which way to put it
    public double[] newGetThetaAndR(Mat img, double robot_offset_y, double tape_offset_x, double tape_offset_y, double height){
        double[] rTheta = {-1, -1, -1, -1}; // for theta1, Dy, and Dx
        double[] center = findCenter(img);
        if(center[0] == -1 && center[1] == -1){
            return rTheta;
        }

        double pixel_x = center[0];
        double pixel_y = center[1];
        pixel_x *= 1280.0/320.0;
        pixel_y *= 720.0/240.0;

        // pixel_x *= 1280.0/160.0;
        // pixel_y *= 720.0/120.0;


        //double pixel_delta_x = -(img.width() / 2 - pixel_x);
        //double pixel_delta_y = img.height() / 2 - pixel_y;
        double pixel_delta_x = -(1280.0 / 2 - pixel_x);
        double pixel_delta_y = 720.0/2 - pixel_y;

        double camera_r = convert_dist(Math.sqrt(Math.pow(pixel_delta_x,2) + Math.pow(pixel_delta_y, 2)), height);
        double camera_theta = (pixel_delta_x == 0 ? Math.PI / 2 * Math.signum(pixel_delta_y) : Math.atan(pixel_delta_y / pixel_delta_x)); //for negative pixel_delta_x, should take return a negative angle

        double slope = find_longer_line(img);
        if(slope == -1){
            return rTheta;      
        }
        double cameraToTape_theta = -getCameraToTapeTheta(slope); //counterclockwise is negative

        // aliases; new names are the ones used in the diagram
        double theta1 = -cameraToTape_theta;
        double theta2 = camera_theta;
        double r = camera_r;
        double d = robot_offset_y;
        double deltaX = tape_offset_x;
        double deltaY = tape_offset_y; // note that the tape_offset_y is the forward/backwards distance from the tape still

        // theta2 in Math.sin should always be positive
        double R = Math.sqrt(d*d + r*r + 2*d*r*Math.sin(Math.abs(theta2)));

        double theta3 = Math.acos((d*d + R*R - r*r) / (2*d*R));
        theta3 *= Math.signum(theta2); // theta3 takes the sign of theta2
        double Rx = R * Math.sin(theta3 - theta1); // it's NEGATIVE theta1, check diagram (theta1 is turn angle from vertical)
        double Ry = R * Math.cos(theta3 - theta1);
        double Dx = Rx - deltaX;
        // hard to find which way to put the Dy (add or subtract) and we probably don't need it anyway
        //double Dy = Ry + deltaY; // + or - deltaY depends on which way we are oriented
        double Dy = Ry;

        rTheta[0] = theta1;
        rTheta[1] = Dy;
        rTheta[2] = Dx;
        rTheta[3] = theta3;
        
        visionValuesDebug(R, Rx, Ry, Dx, Dy, r, d, theta1, theta2, theta3);

        return rTheta;
    }
    public void visionValuesDebug(double R1, double Rx1, double Ry1, double Dx1, double Dy1, double r1, double d1, double theta1, double theta2, double theta3) {
        SmartDashboard.putNumber("R", R1);
        SmartDashboard.putNumber("Rx", Rx1);
        SmartDashboard.putNumber("Ry", Ry1);
        SmartDashboard.putNumber("Dx", Dx1);
        SmartDashboard.putNumber("Dy", Dy1);
        SmartDashboard.putNumber("r", r1);
        SmartDashboard.putNumber("d", d1);
        SmartDashboard.putNumber("2dr*cos(theta2+90)", 2*d1*r1*Math.cos(theta2 + Math.PI/2));
        SmartDashboard.putNumber("theta1 (degrees)", Math.toDegrees(theta1));
        SmartDashboard.putNumber("theta2 (degrees)", Math.toDegrees(theta2));
        SmartDashboard.putNumber("theta3 (degrees)", Math.toDegrees(theta3));
        SmartDashboard.putNumber("theta3 - theta1 (degrees)", Math.toDegrees(theta3 - theta1));
    }

    // returns -1, -1 if no contour found postfilter
    // note that convention here is: negative theta is clockwise (x negative -> theta is also negative)
    public double[] get_final_R_theta(Mat img, double robot_offset_x, double robot_offset_y, double tape_offset_x, double tape_offset_y, double height){
        double[] rThe = {-1, -1};
        double tape_offset_r = Math.sqrt(Math.pow(tape_offset_x, 2) + Math.pow(tape_offset_y, 2));
        double tape_offset_theta = (tape_offset_y == 0 ? Math.PI / 2 * Math.signum(tape_offset_x) : Math.atan2(tape_offset_x, tape_offset_y));

        double[] center = findCenter(img);
        if(center[0] == -1 && center[1] == -1){
            return rThe;
        }
        double pixel_x = center[0];
        double pixel_y = center[1];
        pixel_x *= 1280.0/320.0;
        pixel_y *= 720.0/240.0;

        double pixel_delta_x = -(img.width() / 2 - pixel_x);
        double pixel_delta_y = img.height() / 2 - pixel_y;

        double camera_r = convert_dist(Math.sqrt(Math.pow(pixel_delta_x,2) + Math.pow(pixel_delta_y, 2)), height);
        double camera_theta = (pixel_delta_y == 0 ? Math.PI / 2 * Math.signum(pixel_delta_x): Math.atan2(pixel_delta_x, pixel_delta_y)); //for negative pixel_delta_x, should take return a negative angle

        double camera_delta_x = camera_r * Math.sin(camera_theta);
        double camera_delta_y = camera_r * Math.cos(camera_theta);

        double slope = find_longer_line(img);
        if(slope == -1){
            return rThe;            
        }

        double cameraToTape_theta = -getCameraToTapeTheta(slope); //counterclockwise is negative

        SmartDashboard.putNumber("AngleToTapeSide", cameraToTape_theta * 180.0 / Math.PI);

        
        // cos is x and sin is y intentionally based on empirical (actually swapped back again)
        // not sure if this is because camera to tape theta is actually returning the 90 degrees off angle
        // double tape_delta_x = tape_offset_r * Math.sin(cameraToTape_theta + tape_offset_theta);
        // double tape_delta_y = tape_offset_r * Math.cos(cameraToTape_theta + tape_offset_theta);

        // only using case where tapeoffsetx is 0 (offset is collinear to the tape)
        double tape_delta_x = tape_offset_y * Math.sin(cameraToTape_theta);
        double tape_delta_y = tape_offset_y * Math.cos(cameraToTape_theta);

        SmartDashboard.putNumber("tape offset theta", tape_offset_theta);
        SmartDashboard.putNumber("cameraToTape_theta", cameraToTape_theta);
        SmartDashboard.putNumber("robot_offset_y", robot_offset_y);
        SmartDashboard.putNumber("camera_delta_y", camera_delta_y);
        SmartDashboard.putNumber("tape_delta_y", tape_delta_y);
        SmartDashboard.putNumber("robot_offset_x", robot_offset_x);
        SmartDashboard.putNumber("camera_delta_x", camera_delta_x);
        SmartDashboard.putNumber("tape_delta_x", tape_delta_x);
        SmartDashboard.putNumber("tape_offset_r", tape_offset_r);

        double delta_y = robot_offset_y + camera_delta_y + tape_delta_y;
        double delta_x = robot_offset_x + camera_delta_x + tape_delta_x;
        rThe[0] =  Math.sqrt(Math.pow(delta_x, 2) + Math.pow(delta_y, 2));
        rThe[1] = (delta_y == 0 ? Math.PI * Math.signum(pixel_x) : Math.atan2(delta_x, delta_y));

        return rThe;
    }

    // undistort + get_final_r_theta
    // CURRENTLY, UNDISTORT IS NOT BEING USED (find a way to make it take less time)
    // returns -1, -1 if no contours or more than one contour found
    public double[] get_move_to_correct_point(Mat img,double robot_offset_x, double robot_offset_y, double tape_offset_x, double tape_offset_y, double height) throws FileNotFoundException{
        
        Mat mapx = new Mat(720, 1280, CvType.CV_64FC1);
        Mat mapy = new Mat(720, 1280, CvType.CV_64FC1);

        Scanner in = new Scanner(new File(Filesystem.getDeployDirectory() + "/mapx_values.csv"));
        //SmartDashboard.putString("Scanner init:", "yes 1");
        in.useDelimiter(",");
        for(int row= 0; row <720; row++){
            for(int col = 0; col < 1280; col++){
                float num = in.nextFloat();
                mapx.put(row, col, num);
            }
        }
        in = new Scanner(new File(Filesystem.getDeployDirectory() + "/mapy_values.csv"));
        //SmartDashboard.putString("Scanner init:", "yes 2");
        in.useDelimiter(",");
        for(int row = 0; row < 720; row++){
            for(int col = 0; col < 1280; col++ ){
                float num = in.nextFloat();
                mapy.put(row, col, num);
            }
            //SmartDashboard.putNumber("loops", row);
        }
        
        //SmartDashboard.putString("Progress:", "Finished mapx/mapy init"); 
        //SmartDashboard.putString("Progress:", "Finished img undistort");
        double[] outputRTheta = get_final_R_theta(img, robot_offset_x,  robot_offset_y,  tape_offset_x,  tape_offset_y,  height);
        return outputRTheta;
    }

    // returns -1 if no contour detected
    public double get_alignedToTape_theta(MatOfPoint img) {
        double slope = find_longer_line(img);

        if(slope == -1){
            return slope;
        }

        double turn_theta = getCameraToTapeTheta(slope);
        return turn_theta;
    }

    // returns r3 (distance from first endpoint to center of tape) and final theta to turn
    // r3 in inches, finalTheta in radians, NOT degrees
    public double[] getSecondRTheta(double r1, double r2, double theta1_2, double theta1){
        double theta2 = theta1_2 - theta1;
        double r3 = Math.sqrt(r1 * r1 + r2 * r2 + 2*r1*r2*Math.cos(theta2));
        // random values in case r3 is 0
        if(r3 <= 0.0){
            double[] output = {0, 0};
            return output;
        }
        double finalTheta = Math.PI - Math.asin(r1 * Math.sin(theta2) / r3);
        double[] output = {r3, finalTheta};
        return output;
    }


//Tup class used in find longer line
    public class Tup implements Comparable< Tup >{
        public Double distance;
        public double slope;
        public Mat point1;
        public Mat point2;

        public Tup(Double distance, double slope, Mat point1, Mat point2){
            this.distance = distance;
            this.slope = slope;
            this.point1 = point1;
            this.point2 = point2;
        }
        public Tup(){
            this.distance = null;
            this.slope = 0;
            this.point1 = null;
            this.point2 = null;
        }

        public int compareTo(Tup other) {
            return this.distance.compareTo(other.distance);
        }
    }
}