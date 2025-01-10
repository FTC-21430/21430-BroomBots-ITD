package org.firstinspires.ftc.teamcode.Resources;

import android.media.Image;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

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

    final int minContourLength = 58;
    final int maxContourLength = 135;
    final double minDistance = 13;


// true only if we found a sample the last time we checked
    private boolean foundSample = false;

    public SampleDetection(){

    }

    public void processImage(Mat img){
        foundSample = false;

        Mat cameraMatrix = new Mat()
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
