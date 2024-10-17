package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.systems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class ITDbot extends Robot {
    //TODO tune these numbers for ITDbot, these are from Zirka
    public static double derivativeConstantAngleDef = 0.0015;
    public static double proportionalConstantAngleDef = 0.02;
    
    // overrides the autoMoveTo method in Robot.java to add in more year specific things.
    @Override
    public void autoMoveTo(double targetX, double targetY, double robotAngle, double targetCircle){
        while(distanceCircle(targetX, targetY) > targetCircle &&opModeActive){
            //put all control things that teleop has in here
            pathFollowing.followPath(odometry.getRobotX(),odometry.getRobotY(),odometry.getRobotAngle());
            driveTrain.setDrivePower(pathFollowing.getPowerF(), pathFollowing.getPowerS(), anglePID.getPower(), odometry.getRobotAngle());
        }
    }
    
    public void Init(HardwareMap hardwareMap){
        super.init(hardwareMap, 0, 0, 0);
        derivativeConstantAngle = derivativeConstantAngleDef;
        proportionalConstantAngle = proportionalConstantAngleDef;
    }
}
