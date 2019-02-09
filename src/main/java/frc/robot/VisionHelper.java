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
    public void visionHelp(Mat mapx, Mat mapy) {
   Mat img = Imgcodecs.imread(getClass().getResource("/fname.png").getPath()); 
   Mat gray = null;
   Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);
   Mat dst = null;
   Imgproc.remap(img, dst, mapx, mapy, Imgproc.INTER_LINEAR); 
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
//########################################## 2.3b: ANGLE FROM TAPE SIDE TO CAMERA FACING #####################################################

public double getCameraToTapeTheta(double m){
    //y = y0 + m(x - x0)
    //using one point x = x0, another point x = x0 + 100
    //find two points on the line. camera forward defines the y of the image, so finding the angle from the camera line is the same as finding the
    //angle from just the y axis. After two points on the line are found, find delta y and delta x, and then get atan.

    //ARC TAN OF THE SLOPE BASICALLY 
    return Math.atan(m);
    
}


}
