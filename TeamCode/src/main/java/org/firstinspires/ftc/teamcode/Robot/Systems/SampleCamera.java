package org.firstinspires.ftc.teamcode.Robot.Systems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Resources.SampleDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class SampleCamera {
    private double cameraXRobot;
    private double cameraYRobot;
    private double cameraZRobot;
    private double cameraDistance;
    private final double CAMERA_YAW_ROBOT = 90.0;
    private final double PIVOT_OFFSET = 6;
    private final double TUBE_LENGTH = 16.75;
    private final double CAMERA_OFFSET = 1.50;
    private final double CHASSIS_HEIGHT = 5.73;

    // what we measured from the robot
    private final double Y_LENS_OFFSET_MEASURED = 21.6+ Math.sqrt(3);

    private Telemetry telemetry = null;

    private VisionPortal samplePortal;

    private SampleDetectionProcessor samples;

    private double sample2RobotX;
    private double sample2RobotY;
    private double sample2RobotYaw;


    public SampleCamera(HardwareMap hardwareMap, Telemetry telemetry) {
       this.telemetry = telemetry;

       samples = new SampleDetectionProcessor.Builder()
               .createSampleDetector(telemetry)
                .build();

       VisionPortal.Builder builder = new VisionPortal.Builder();

       builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

       builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

       builder.setCameraResolution(new Size(1920,1080));

       builder.addProcessor(samples);

       samplePortal = builder.build();

    }

    public void findCameraPosRelativePosition(double shoulderAngle, double extension){
        if (didWeFindOne()) {

            cameraXRobot = CAMERA_OFFSET;
            cameraYRobot = Y_LENS_OFFSET_MEASURED;
            cameraZRobot = CHASSIS_HEIGHT + ((TUBE_LENGTH + extension) * Math.sin(shoulderAngle));
            cameraDistance = Math.hypot(cameraXRobot, cameraYRobot);

            sample2RobotX = -samples.getFoundSamplePositionY() + cameraXRobot;
            sample2RobotY = samples.getFoundSamplePositionX() + cameraYRobot;
            sample2RobotYaw = samples.getFoundSamplePositionYaw() - CAMERA_YAW_ROBOT;
        }
    }
    public double getSampleY(){
        return sample2RobotY;
    }
    public double getSampleX(){
        return sample2RobotX;
    }
    public double getSampleYaw(){
        return sample2RobotYaw;
    }


    public void findYellowSample(){
        samples.enableSampleDetection();
        samples.setFilterToYellow();
    }
    public void findRedSample(){
        samples.enableSampleDetection();
        samples.setFilterToRed();
    }
    public void findBlueSample(){
        samples.enableSampleDetection();
        samples.setFilterToBlue();
    }


    public void startDetection(){
        samples.enableSampleDetection();
    }
    public void stopDetection(){
        samples.disableSampleDetection();
    }

    public boolean isActive(){
        return samples.update;
    }

    public boolean didWeFindOne(){
        return samples.isFoundSample();
    }


}
