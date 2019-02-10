package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
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
    /*public Mat undistort(Mat img, Mat mapx, Mat mapy) {
  // Mat img = Imgcodecs.imread(getClass().getResource("/fname.png").getPath()); 
   Mat gray = null;
   Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);
   Mat dst = null;
   Imgproc.remap(img, dst, mapx, mapy, Imgproc.INTER_LINEAR); 
    return dst;
    }

    public double[] findCenter(MatOfPoint img){
    //[x,y]
    double[] centerCoor = new double[2];
    GripPipeline pipeline = new GripPipeline();
    pipeline.process(img);
    Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
    centerCoor[0] = r.x + (r.width / 2);
    centerCoor[1]= r.y + (r.height/2);
    return centerCoor;
    }
    public double convert_dist(double pixel_dist, double height){
    return 0.0001 * (9.081 * height * pixel_dist);
    }

    class Tup:
    def __init__(self, distance, slope, point1, point2):
        self.distance = distance
        self.slope = slope
        self.point1 = point1
        self.point2 = point2

// returns slope of line
    public tuple? find_longer_line(MatOfPoint img){
    GripPipeline pipeline = new GripPipeline();
    pipeline.process(img);
    contours = pipeline.filterContoursOutput().get(0);

    //returns m, y0, and x0 of longer line
    RotatedRect rc = Imgproc.minAreaRect(contours[0]);
    box = Imgproc.boxPoints(rc, img); 

    tups = [] #list of tuples

    
for (int i : enumerate(box)){
    for (int j : enumerate(box)){
            if (i < j){}
                ydiff = p2[1] - p1[1]// difference in y coords
                xdiff = p2[0] - p1[0] //difference in x coords
                distance = sqrt(xdiff * xdiff+ ydiff *ydiff) //distance formula to find distance between 2 points
                slope = ydiff / (xdiff * 1.0);
                tups.append(Tup(distance, slope, p1, p2)) #add in the tuple into the list 
    }
    }
}

    tups.sort(key=distance)
    return tups[2].slope
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
    double pixel_delta_x = img.shape[0] / 2 - pixel_x;
    double pixel_delta_y = img.shape[1] / 2 - pixel_y;

    double camera_r = convert_dist(Math.sqrt(Math.pow(pixel_delta_x,2)+Math.pow(pixel_delta_y, 2)), height);
    double camera_theta = Math.atan(pixel_delta_y/pixel_delta_x);//for negative pixel_delta_x, should take return a negative angle

    double camera_delta_x = camera_r * Math.cos(camera_theta);
    double camera_delta_y = camera_r * Math.sin(camera_theta);

    double cameraToTape_theta = get_cameraToTape_Theta(find_longer_line(img));

    double tape_delta_x = tape_offset_r * Math.cos(cameraToTape_theta + tape_offset_theta);
    double tape_delta_y = tape_offset_r * Math.sin(cameraToTape_theta + tape_offset_theta);

    double delta_y = robot_offset_y + camera_delta_y + tape_delta_y;
    double delta_x = robot_offset_x + camera_delta_x + tape_delta_x;
    Math.sqrt(Math.pow(tape_offset_x,2)+Math.pow(tape_offset_y,2));
    rThe[0] =  Math.sqrt(Math.pow(delta_x,2)+Math.pow(delta_y,2));
    rThe[1]= Math.atan(delta_y/delta_x);

    return rThe;

}
//"path to image" = placeholder
Mat img = Imgcodecs.imread("path to image")
return
robot_offset_x = "measure this";
robot_offset_y = "measure this as well";
tape_offset_x = "this too";
tape_offset_y =  "this three";
height = "this four";*/

}
