package frc.robot;

import org.opencv.core.*;
import org.opencv.core.Core.*;
//import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

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
