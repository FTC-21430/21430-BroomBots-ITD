package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

public class ITDbot extends Robot {
    //TODO tune these numbers for ITDbot, need to be updated for Into the deep
    public static double derivativeConstantAngleDef = 0.0015;
    public static double proportionalConstantAngleDef = 0.02;
    public SpampleArm arm;
 
    public void Init(HardwareMap hardwareMap, Telemetry telemetry){
        super.init(hardwareMap, telemetry,0, 0, 0);
        arm=new SpampleArm(hardwareMap);
}
    // overrides the autoMoveTo method in Robot.java to add in more year specific things.
    @Override
    public void autoMoveTo(double targetX, double targetY, double robotAngle, double targetCircle){
        while(distanceCircle(targetX, targetY) > targetCircle&&opModeActive){
            pathFollowing.setTargetPosition(targetX,targetY);
            anglePID.setTarget(robotAngle);
            //put all control things that teleop has in here
            anglePID.update(odometry.getRobotAngle());
            pathFollowing.followPath(odometry.getRobotX(),odometry.getRobotY(),odometry.getRobotAngle());
            driveTrain.setDrivePower(pathFollowing.getPowerF(), pathFollowing.getPowerS(), anglePID.getPower(), odometry.getRobotAngle());
        }
    
        derivativeConstantAngle = derivativeConstantAngleDef;
        proportionalConstantAngle = proportionalConstantAngleDef;
    }
}
