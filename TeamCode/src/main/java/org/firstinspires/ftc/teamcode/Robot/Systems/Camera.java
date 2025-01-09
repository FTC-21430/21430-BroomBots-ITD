package org.firstinspires.ftc.teamcode.Robot.Systems;

import org.firstinspires.ftc.teamcode.Resources.InverseKinematics;

public class Camera {
    private double cameraXRobot;
    private double cameraYRobot;
    private double cameraZRobot;
    private double cameraDistance;
    private double cameraYawRobot;
    private final double PIVOT_OFFSET = 2.55;
    private final double TUBE_LENGTH = 16.75;
    private final double CAMERA_OFFSET = 1.00; //need to check value (also possibly negative)
    private final double CHASSIS_HEIGHT = 5.73;

    public Camera(double shoulderAngle, double extension) {
       cameraXRobot = CAMERA_OFFSET;
       cameraYRobot = Math.cos(shoulderAngle) * (TUBE_LENGTH + extension) - PIVOT_OFFSET;
       cameraZRobot = CHASSIS_HEIGHT + (TUBE_LENGTH+extension) * Math.sin(shoulderAngle);
       cameraDistance = Math.hypot(cameraXRobot, cameraYRobot);
       cameraYawRobot = Math.atan2(cameraYRobot,cameraXRobot);
    }

    private double getCameraXRobot(){
        return cameraXRobot;
    }
    private double getCameraYRobot() {
        return cameraYRobot;
    }

    private double getCameraZRobot(double shoulderAngle, double extension){
        return cameraZRobot;
    }

    private double getCameraDistance(){
        return cameraDistance;
    }

    private double getCameraYawRobot(){
        return cameraYawRobot;
    }
}
