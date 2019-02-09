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
   Mat gray;
   Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY, 0);
   Mat dst;
   Imgproc.remap(img, dst, mapx, mapy, Imgproc.INTER_LINEAR); 
    }

    public double[] findCenter(MatOfPoint img){
    //[x,y]
    double[] centerCoor = new double[2];
    GripPipeline pipeline = new GripPipeline();
    pipeline.process(undistorted);
    Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
    centerCoor[0] = r.x + (r.width / 2);
    centerCoor[1]= r.y + (r.height/2);
    return centerCoor;
    }
}
