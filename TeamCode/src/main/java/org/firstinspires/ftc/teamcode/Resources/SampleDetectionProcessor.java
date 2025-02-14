package org.firstinspires.ftc.teamcode.Resources;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionProcessor;

public abstract class SampleDetectionProcessor implements VisionProcessor {

    // The position data about where the sample we want to grab is RELATIVE TO CAMERA, These are in polar coordinates
    public double foundSamplePositionThetaBuffer = 0;
    public double foundSamplePositionRadiusBuffer = 0;
    public double foundSamplePositionYawBuffer = 0;

    private double foundSamplePositionTheta = 0;
    private double foundSamplePositionRadius = 0;
    private double foundSamplePositionYaw = 0;


    public boolean update= false;

    // true only if we found a sample the last time we checked
    public boolean foundSample = false;
    public boolean calculating = false;

    public int colorMode = 2;
    public static class Builder
    {
        SampleDetectionPipeline sampleDetectionPipeline;

        public Builder createSampleDetector(Telemetry telemetry){
            sampleDetectionPipeline = new SampleDetectionPipeline(telemetry);
            return this;
        }

        public SampleDetectionProcessor build()
        {
            return sampleDetectionPipeline;
        }
    }

    public double getFoundSamplePositionTheta(){
        return foundSamplePositionTheta;
    }
    public double getFoundSamplePositionRadius(){
        return foundSamplePositionRadius;
    }
    public double getFoundSamplePositionYaw(){
        return foundSamplePositionYaw;
    }
    public boolean isFoundSample(){
        return foundSample;
    }

    public void enableSampleDetection(){
        calculating = true;
        foundSample = false;
        update = true;
    }
    public void disableSampleDetection(){

//        foundSample = false;
        update = false;
    }


    public void setFilterToYellow() {

        colorMode = 0;
    }


    public void setFilterToRed(){

        colorMode = 1;
    }

    public void setFilterToBlue(){

        colorMode = 2;
    }

    public void acceptBuffer(){
        foundSamplePositionRadius = foundSamplePositionRadiusBuffer;
        foundSamplePositionTheta = foundSamplePositionThetaBuffer;
        foundSamplePositionYaw = foundSamplePositionYawBuffer;
    }
    public void beginProccessing(){
        calculating = false;
    }
    public boolean getIfDone(){
        return calculating;
    }

}
