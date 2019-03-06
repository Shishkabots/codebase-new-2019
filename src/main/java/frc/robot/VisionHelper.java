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
        return Math.atan(m);
        
    }

    // returns -1, -1 if no contour found postfilter
    // note that convention here is: negative theta is clockwise (x negative -> theta is also negative)
    public double[] get_final_R_theta(Mat img,double robot_offset_x, double robot_offset_y, double tape_offset_x, double tape_offset_y, double height){
        double[] rThe = {-1, -1};
        double tape_offset_r = Math.sqrt(Math.pow(tape_offset_x, 2) + Math.pow(tape_offset_y, 2));
        double tape_offset_theta = (tape_offset_y == 0 ? Math.PI / 2 * Math.signum(tape_offset_x) : Math.atan(tape_offset_x / tape_offset_y));

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
        double camera_theta = (pixel_delta_y == 0 ? Math.PI * Math.signum(pixel_delta_x): Math.atan(pixel_delta_x/pixel_delta_y)); //for negative pixel_delta_x, should take return a negative angle

        double camera_delta_x = camera_r * Math.sin(camera_theta);
        double camera_delta_y = camera_r * Math.cos(camera_theta);

        double slope = find_longer_line(img);
        if(slope == -1){
            return rThe;            
        }
        double cameraToTape_theta = getCameraToTapeTheta(slope);

        double tape_delta_x = tape_offset_r * Math.sin(cameraToTape_theta + tape_offset_theta);
        double tape_delta_y = tape_offset_r * Math.cos(cameraToTape_theta + tape_offset_theta);

        double delta_y = robot_offset_y + camera_delta_y + tape_delta_y;
        double delta_x = robot_offset_x + camera_delta_x + tape_delta_x;
        rThe[0] =  Math.sqrt(Math.pow(delta_x, 2) + Math.pow(delta_y, 2));
        rThe[1] = (delta_y == 0 ? Math.PI * Math.signum(pixel_x) : Math.atan(delta_x/delta_y));

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

        //img = undistort(img, mapx, mapy); 
        //SmartDashboard.putString("Progress:", "Finished img undistort");
        double[] outputRTheta = get_final_R_theta(img, robot_offset_x,  robot_offset_y,  tape_offset_x,  tape_offset_y,  height);
        return outputRTheta;
    }

    // returns -1 if no contour detected
    public double get_alignedToTape_theta(MatOfPoint img) {
        //Mat img_new = Imgcodecs.imread("path to image-new?");
        double slope = find_longer_line(img);

        if(slope == -1){
            return slope;
        }

        double turn_theta = getCameraToTapeTheta(slope);
        return turn_theta;
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