/*
IMPORTANT!!!!!!!!!
There are changes in this grippipeline, such as modified contour filtering, printing number of
contours prefilter/postfilter, and printing contour image. If you swap out the pipeline, DON'T
JUST DELETE AND REPLACE. please copy over changes thank you much appreciated.
*/

package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import edu.wpi.first.wpilibj.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;

/**
* GripPipeline class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class GripPipeline implements VisionPipeline {

	//Outputs
	private Mat resizeImageOutput = new Mat();
	private Mat hsvThresholdOutput = new Mat();
	private Mat cvDilateOutput = new Mat();
	private Mat cvErodeOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
	
    CvSource contoursPrefilter = Robot.out;

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	@Override	
	public void process(Mat source0) {
		// Step Resize_Image0:
		Mat resizeImageInput = source0;
		double resizeImageWidth = 320.0;
		double resizeImageHeight = 240.0;
		int resizeImageInterpolation = Imgproc.INTER_CUBIC;
		resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput);

		// Step HSV_Threshold0:
		Mat hsvThresholdInput = resizeImageOutput;
		double[] hsvThresholdHue = {0.0, 180.0};
		double[] hsvThresholdSaturation = {0.0, 52.21534116794114};
		double[] hsvThresholdValue = {216.5434294550931, 255.0};
		hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

		// Step CV_dilate0:
		Mat cvDilateSrc = hsvThresholdOutput;
		Mat cvDilateKernel = new Mat();
		Point cvDilateAnchor = new Point(-1, -1);
		double cvDilateIterations = 3.0;
		int cvDilateBordertype = Core.BORDER_CONSTANT;
		Scalar cvDilateBordervalue = new Scalar(-1);
		cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);

		// Step CV_erode0:
		Mat cvErodeSrc = cvDilateOutput;
		Mat cvErodeKernel = new Mat();
		Point cvErodeAnchor = new Point(-1, -1);
		double cvErodeIterations = 3.0;
		int cvErodeBordertype = Core.BORDER_CONSTANT;
		Scalar cvErodeBordervalue = new Scalar(-1);
		cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

		// Step Find_Contours0:
		Mat findContoursInput = cvErodeOutput;
		boolean findContoursExternalOnly = false;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		// SmartDashboard.putString("Sink attached to contoursPrefilter: ", contoursPrefilter.enumerateSinks()[0].getName());
		// SmartDashboard.putString("Sink attached to contoursPrefilter description: ", contoursPrefilter.enumerateSinks()[0].getDescription());
		// SmartDashboard.putNumber("sinks attached to contoursPrefilter count: ", contoursPrefilter.enumerateSinks().length);
		// SmartDashboard.putString("Sink type:", contoursPrefilter.enumerateSinks()[0].getKind().toString());


		SmartDashboard.putNumber("Contours prefilter", findContoursOutput.size());
		// Point[] pen0r = findContoursOutput.get(0).toArray();
		// SmartDashboard.putNumber("Pen0r size", pen0r.length);
		// SmartDashboard.putNumber("Pen0r 1", pen0r[0].x);
		// SmartDashboard.putNumber("Pen0r 2", pen0r[0].y);

		Imgproc.drawContours(resizeImageOutput, findContoursOutput, -1, new Scalar(0, 255, 0));
		// SmartDashboard.putNumber("Suze", resizeImageOutput.width());
		// SmartDashboard.putNumber("Soze", resizeImageOutput.height());
		contoursPrefilter.putFrame(resizeImageOutput);

		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
		double filterContoursMinArea = 1300.0;
		double filterContoursMinPerimeter = 200.0;
		double filterContoursMinWidth = 0.0;
		double filterContoursMaxWidth = 1000.0;
		double filterContoursMinHeight = 0.0;
		double filterContoursMaxHeight = 1000.0;
		double[] filterContoursSolidity = {0, 100};
		double filterContoursMaxVertices = 1000.0;
		double filterContoursMinVertices = 0.0;
		double filterContoursMinRatio = 0.0;
		double filterContoursMaxRatio = 1000.0;

		double maxArea = 1000000.0; // we don't want this constraint
		double maxPerimeter = 400.0;
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput, maxArea, maxPerimeter);

		SmartDashboard.putNumber("Contours postfilter", filterContoursOutput.size());
	}

	/**
	 * This method is a generated getter for the output of a Resize_Image.
	 * @return Mat output from Resize_Image.
	 */
	public Mat resizeImageOutput() {
		return resizeImageOutput;
	}

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * @return Mat output from HSV_Threshold.
	 */
	public Mat hsvThresholdOutput() {
		return hsvThresholdOutput;
	}

	/**
	 * This method is a generated getter for the output of a CV_dilate.
	 * @return Mat output from CV_dilate.
	 */
	public Mat cvDilateOutput() {
		return cvDilateOutput;
	}

	/**
	 * This method is a generated getter for the output of a CV_erode.
	 * @return Mat output from CV_erode.
	 */
	public Mat cvErodeOutput() {
		return cvErodeOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * @return ArrayList<MatOfPoint> output from Find_Contours.
	 */
	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	}

	/**
	 * This method is a generated getter for the output of a Filter_Contours.
	 * @return ArrayList<MatOfPoint> output from Filter_Contours.
	 */
	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
	}


	/**
	 * Scales and image to an exact size.
	 * @param input The image on which to perform the Resize.
	 * @param width The width of the output in pixels.
	 * @param height The height of the output in pixels.
	 * @param interpolation The type of interpolation.
	 * @param output The image in which to store the output.
	 */
	private void resizeImage(Mat input, double width, double height,
		int interpolation, Mat output) {
		Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param val The min and max value
	 * @param output The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
	    Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
			new Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Expands area of higher value in an image.
	 * @param src the Image to dilate.
	 * @param kernel the kernel for dilation.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the dilation.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
	int borderType, Scalar borderValue, Mat dst) {
		if (kernel == null) {
			kernel = new Mat();
		}
		if (anchor == null) {
			anchor = new Point(-1,-1);
		}
		if (borderValue == null){
			borderValue = new Scalar(-1);
		}
		Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Expands area of lower value in an image.
	 * @param src the Image to erode.
	 * @param kernel the kernel for erosion.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the erosion.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
		int borderType, Scalar borderValue, Mat dst) {
		if (kernel == null) {
			kernel = new Mat();
		}
		if (anchor == null) {
			anchor = new Point(-1,-1);
		}
		if (borderValue == null) {
			borderValue = new Scalar(-1);
		}
		Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
	 * @param input The image on which to perform the Distance Transform.
	 * @param type The Transform.
	 * @param maskSize the size of the mask.
	 * @param output The image in which to store the output.
	 */
	private void findContours(Mat input, boolean externalOnly,
		List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}


	/**
	 * Filters out contours that do not meet certain criteria.
	 * @param inputContours is the input list of contours
	 * @param output is the the output list of contours
	 * @param minArea is the minimum area of a contour that will be kept
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept
	 * @param minWidth minimum width of a contour
	 * @param maxWidth maximum width
	 * @param minHeight minimum height
	 * @param maxHeight maximimum height
	 * @param Solidity the minimum and maximum solidity of a contour
	 * @param minVertexCount minimum vertex Count of the contours
	 * @param maxVertexCount maximum vertex Count
	 * @param minRatio minimum ratio of width to height
	 * @param maxRatio maximum ratio of width to height
	 */
	private void filterContours(List<MatOfPoint> inputContours, double minArea,
		double minPerimeter, double minWidth, double maxWidth, double minHeight, double
		maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
		minRatio, double maxRatio, List<MatOfPoint> output, double maxArea, double maxPerimeter) {
		final MatOfInt hull = new MatOfInt();
		output.clear();
		//operation
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			final double area = Imgproc.contourArea(contour);
			if (area < minArea || area > maxArea) continue;
			double perimeter = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
			if (perimeter < minPerimeter || perimeter > maxPerimeter) continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int)hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
			final double ratio = bb.width / (double)bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.add(contour);
		}
	}




}

