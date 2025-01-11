package org.firstinspires.ftc.teamcode.Resources;

import android.media.Image;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is designed for you to use it an image from the camera on the end of the arm and
 * it will return the position and rotation of the best sample to grab it could find
 * ( the one that is closest to the center of the image)
 */
public class SampleDetection {

    // The position data about where the sample we want to grab is RELATIVE TO CAMERA
private double foundSamplePositionX = 0;
private double foundSamplePositionY = 0;
private double foundSamplePositionYaw = 0;

// Thresholding values for Yellow Samples

    final double YellowHL = 12;
    final double YellowHH = 61;
    final double YellowSL = 119;
    final double YellowSH = 255;
    final double YellowVL = 186;
    final double YellowVH = 255;

// Thresholding values for the Red Samples

    final double RedHL = 0;
    final double RedHH = 0;
    final double RedSL = 0;
    final double RedSH = 0;
    final double RedVL = 0;
    final double RedVH = 0;

// Thresholding values for the Blue Samples

    final double BlueHL = 0;
    final double BlueHH = 0;
    final double BlueSL = 0;
    final double BlueSH = 0;
    final double BlueVL = 0;
    final double BlueVH = 0;

// Edge detection constants

    final double CannyLow = 272;
    final double CannyHigh = 27;

// Contour filtering constants

    final int minContourLength = 290;
    final int maxContourLength = 675;
    final double minDistance = 65;

// unit conversions
    final double PIX2INCHES = 4/200;

// Other constants

    final double CROPPING_FACTOR = 2;
    final double EXTRA_CROPPING_X = 25;
    final double EXTRA_CROPPING_Y = 200;

// true only if we found a sample the last time we checked
    private boolean foundSample = false;

    public SampleDetection(){

    }

    public void findYellowSamples(Mat img){
        foundSample = false;

        Mat corrected = undistortImage(img);

        Mat blurred = null;

        Imgproc.GaussianBlur(corrected, blurred, new Size(3,3),5);

        Mat cropped = crop(blurred);

        Mat hsv = convertColorSpace(cropped);
        Mat thresholded = thresholdImage(hsv, 0);


        Mat kernel = Mat.ones(55,55,0);
        Mat eroded = null;
        Imgproc.erode(thresholded, eroded, kernel);

        Mat baseImage = null;

        Core.subtract(hsv, eroded, baseImage);

        Mat seperationCanny = null;

        Imgproc.Canny(baseImage, seperationCanny, 5, 10);

        Mat dilatedSeperationCanny = null;

        Mat dilationKernal = Mat.ones(4, 4, CvType.CV_32F);

        Imgproc.dilate(seperationCanny, dilatedSeperationCanny, dilationKernal);

        Mat blurredCanny = null;

        Imgproc.GaussianBlur(seperationCanny, blurredCanny, new Size(3,3), 3);

        Mat seperated = null;

        Core.subtract(baseImage, blurredCanny, seperated);

        Mat outlineCanny = null;

        Imgproc.Canny(seperated, outlineCanny, 5, 10);

        Mat outlineBlurred = null;

        Imgproc.GaussianBlur(outlineCanny, outlineBlurred, new Size(3,3), 3);

        List contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(outlineBlurred, contours, hierarchy,Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);



    }

    public void findRedSamples(Mat img){
        foundSample = false;

        Mat corrected = undistortImage(img);

        Mat blurred = null;

        Imgproc.GaussianBlur(corrected, blurred, new Size(3,3),5);

        Mat cropped = crop(blurred);

        Mat hsv = convertColorSpace(cropped);

        Mat thresholded = thresholdImage(hsv, 1);
    }

    public void findBlueSamples(Mat img){
        foundSample = false;

        Mat corrected = undistortImage(img);

        Mat blurred = null;

        Imgproc.GaussianBlur(corrected, blurred, new Size(3,3),5);

        Mat cropped = crop(blurred);

        Mat hsv = convertColorSpace(cropped);

        Mat thresholded = thresholdImage(hsv, 2);
    }

    private Mat thresholdImage(Mat src, int mode){
        Scalar lowerBound = null;
        Scalar upperBound = null;

        Mat dst = null;

        if (mode == 0){
            // yellow

            // create the yellow thresholds
            lowerBound = new Scalar(YellowHL,YellowSL,YellowVL);
            upperBound = new Scalar(YellowHH,YellowSH,YellowVH);

            // threshold the image!
            Core.inRange(src, lowerBound, upperBound, dst);

        }else if(mode == 1){
            //red

            // create the yellow thresholds
            lowerBound = new Scalar(RedHL,RedSL,RedVL);
            upperBound = new Scalar(RedHH,RedSH,RedVH);

            // threshold the image!
            Core.inRange(src, lowerBound, upperBound, dst);

        }else if(mode == 2){
            //blue

            // create the yellow thresholds
            lowerBound = new Scalar(BlueHL,BlueSL,BlueVL);
            upperBound = new Scalar(BlueHH,BlueSH,BlueVH);

            // threshold the image!
            Core.inRange(src, lowerBound, upperBound, dst);

        }
        return dst;
    }

    private Mat crop(Mat src){
        int scaleX = src.cols();
        int scaleY = src.rows();

        int cropScaleX = (int) (scaleX / CROPPING_FACTOR / 2);
        int cropScaleY = (int) (scaleY / CROPPING_FACTOR / 2);

        cropScaleY += EXTRA_CROPPING_Y;
        cropScaleX += EXTRA_CROPPING_X;

        // calculating the part of the image we want to remain
        Rect roi = new Rect(cropScaleX, cropScaleY, (scaleX - cropScaleX) - cropScaleX, (scaleY - cropScaleY)  - cropScaleY);

        Mat cropped = new Mat(src, roi);

        return cropped;
    }

    private Mat convertColorSpace(Mat src){
        Mat hsv = null;
        Imgproc.cvtColor(src, hsv, Imgproc.COLOR_BGR2HSV);
        return hsv;
    }

    private Mat undistortImage(Mat src){
        Mat cameraMatrix = new Mat(3,3,0);

        //These are the values gotten from the camera calibration we did

        // assign the values to the matrix... I could not find a better way through the documentation
        cameraMatrix.put(0,0, 600.01851744);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, 906.817157357);
        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,600.01851744);
        cameraMatrix.put(1,2,516.73047402);
        cameraMatrix.put(2,0,0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);

        // assigning values to the camera distortion coefficients... the only way I know how
        Mat dist_coeffs = new Mat(1,5,0);

        dist_coeffs.put(0,0,0.0115588983608);
        dist_coeffs.put(0,1,-0.0313347203804);
        dist_coeffs.put(0,2,0.00013459478315);
        dist_coeffs.put(0,3,0.000897741867319);
        dist_coeffs.put(0,4,0.00542752872672);

        // create the destination for the undistorted image
        Mat unDistorted = null;

        // un distort
        Calib3d.undistort(src, unDistorted, cameraMatrix, dist_coeffs);
        return unDistorted;
    }

    private Mat rescaleFrame(Mat frame, double scale){
        double width = (int) Math.round(frame.cols() * scale);
        double height = (int) Math.round(frame.rows() * scale);
        Size image_size = new Size(width,height);
        Mat dst = null;
        Imgproc.resize(frame, dst, image_size);
        return dst;
    }

    private int avgInt(int[] numbers){
        double count = 0;
        int i = 0;
        for(int n : numbers){
            count += n;
            i += 1;
        }
        if (i != 0){
            count /= i;
            int avg = (int)count;
            return avg;
        }else{
            return -1;
        }
    }
}
