package org.firstinspires.ftc.teamcode.Robot.Systems;

import org.firstinspires.ftc.teamcode.Resources.InverseKinematics;

public class Camera {
    private double cameraXRobot;
    private double cameraYRobot;
    private double cameraZRobot;
    private double cameraDistance;
    private final double PIVOT_OFFSET = 2.55;
    private final double TUBE_LENGTH = 16.75;
    SpampleArm spampleArm;
    public Camera(SpampleArm spampleArm, double shoulderAngle, double extension, double shoulderOffset) {
       spampleArm = this spampleArm;
    }
//    private double findCameraXRobot() {
//        cameraXRobot = ;
//        return cameraXRobot;
//    }

    private double findCameraXRobot(double CAMERA_OFFSET){
        cameraXRobot = CAMERA_OFFSET;
        return cameraXRobot;
    }
    private double findCameraYRobot(double shoulderAngle, double extension) {
        cameraYRobot = Math.cos(shoulderAngle) * (TUBE_LENGTH + extension) - PIVOT_OFFSET;
        return cameraYRobot;
    }

    private double findCameraZRobot(double shoulderAngle, double extension){
        cameraZRobot = (TUBE_LENGTH+extension) * Math.sin(shoulderAngle);
        return cameraZRobot;
    }

    private double findCameraDistance(){
        cameraDistance = Math.hypot(cameraXRobot, cameraYRobot);
        return cameraDistance;
    }
}
