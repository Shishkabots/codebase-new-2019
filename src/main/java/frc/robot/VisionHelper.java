package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.HashMap;

import  edu.wpi.first.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.core.Core.*;
//import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

public class VisionHelper
{
    public static double[] centerCoor;
    public MatOfPoint undistort(MatOfPoint img, Mat mapx, Mat mapy) {
        // Mat img = Imgcodecs.imread(getClass().getResource("/fname.png").getPath()); 
       // Mat gray = null;
        //Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);
        MatOfPoint dst = null;
        Imgproc.remap(img, dst, mapx, mapy, Imgproc.INTER_LINEAR); 
        return dst;
    }
    public VisionHelper() {

    }

    public double[] findCenter(MatOfPoint img) {
        //[x,y]
        double[] centerCoor = new double[2];
        GripPipeline pipeline = new GripPipeline();
        pipeline.process(img);
         Moments moments = Imgproc.moments(img);
        centerCoor[0] = moments.get_m10() / moments.get_m00();
        centerCoor[1] = moments.get_m01() / moments.get_m00();
        return centerCoor;
    }

    public double convert_dist(double pixel_dist, double height){
        return 0.0001 * (9.081 * height * pixel_dist);
    }
    // returns slope of line
    public double find_longer_line(MatOfPoint img){
        GripPipeline pipeline = new GripPipeline();
        pipeline.process(img);
        MatOfPoint contours = pipeline.filterContoursOutput().get(0);

        //returns m, y0, and x0 of longer line
        MatOfPoint2f myPt = new MatOfPoint2f();
        contours.convertTo(myPt, CvType.CV_32FC2);
        RotatedRect rc = Imgproc.minAreaRect(myPt);
        Mat box = new Mat();    
        Imgproc.boxPoints(rc, box); 

        ArrayList<Tup> tups = new ArrayList<Tup>(); //list of tuple

        
        for (int i = 0; i < box.rows(); i++){
            for (int j = 0; j < box.cols(); j++){
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

//########################################## 2.3b: ANGLE FROM TAPE SIDE TO CAMERA FACING #####################################################

    public double getCameraToTapeTheta(double m){
        //y = y0 + m(x - x0)
        //using one point x = x0, another point x = x0 + 100
        //find two points on the line. camera forward defines the y of the image, so finding the angle from the camera line is the same as finding the
        //angle from just the y axis. After two points on the line are found, find delta y and delta x, and then get atan.

        //ARC TAN OF THE SLOPE BASICALLY 
        return Math.atan(m);
        
    }

    public double[] get_final_R_theta(MatOfPoint img,double robot_offset_x, double robot_offset_y, double tape_offset_x, double tape_offset_y, double height){
        double[] rThe = new double[2];
        double tape_offset_r = Math.sqrt(Math.pow(tape_offset_x,2)+Math.pow(tape_offset_y,2));
        double tape_offset_theta = Math.atan(tape_offset_y / tape_offset_x);

        double[] center = findCenter(img);
        double pixel_x = center[0];
        double pixel_y = center[1];
        double pixel_delta_x = img.width() / 2 - pixel_x;
        double pixel_delta_y = img.height() / 2 - pixel_y;

        double camera_r = convert_dist(Math.sqrt(Math.pow(pixel_delta_x,2)+Math.pow(pixel_delta_y, 2)), height);
        double camera_theta = Math.atan(pixel_delta_y/pixel_delta_x);//for negative pixel_delta_x, should take return a negative angle

        double camera_delta_x = camera_r * Math.cos(camera_theta);
        double camera_delta_y = camera_r * Math.sin(camera_theta);

        double cameraToTape_theta = getCameraToTapeTheta(find_longer_line(img));

        double tape_delta_x = tape_offset_r * Math.cos(cameraToTape_theta + tape_offset_theta);
        double tape_delta_y = tape_offset_r * Math.sin(cameraToTape_theta + tape_offset_theta);

        double delta_y = robot_offset_y + camera_delta_y + tape_delta_y;
        double delta_x = robot_offset_x + camera_delta_x + tape_delta_x;
        Math.sqrt(Math.pow(tape_offset_x,2)+Math.pow(tape_offset_y,2));
        rThe[0] =  Math.sqrt(Math.pow(delta_x,2)+Math.pow(delta_y,2));
        rThe[1]= Math.atan(delta_y/delta_x);

        return rThe;

    }
    public double[] get_move_to_correct_point(MatOfPoint img,double robot_offset_x, double robot_offset_y, double tape_offset_x, double tape_offset_y, double height) {
        //"path to image" = placeholder
        //Mat img = Imgcodecs.imread("path to image"); // don't need to read in img if already passed in

        // need to load from file 
        Mat mapx;
        Mat mapy;

        robot_offset_x = 0; //measure this
        robot_offset_y = 0; //measure this as well
        tape_offset_x = 0; //this too
        tape_offset_y =  0; //this three
        height = 0; //this four

        //img = undistort(img, mapx, mapy); 
        double[] outputRTheta = get_final_R_theta(img,
         robot_offset_x,  robot_offset_y,  tape_offset_x,  tape_offset_y,  height);
        return outputRTheta;
    }

    public double get_alignedToTape_theta(MatOfPoint img) {
        Mat img_new = Imgcodecs.imread("path to image-new?");
        double turn_theta = getCameraToTapeTheta(find_longer_line(img));
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
