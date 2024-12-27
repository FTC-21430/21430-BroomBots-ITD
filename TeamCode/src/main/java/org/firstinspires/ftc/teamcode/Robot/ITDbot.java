package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

public class ITDbot extends Robot {
    //TODO tune these numbers for ITDbot, need to be updated for Into the deep
    public static double derivativeConstantAngleDef = 0.0015;
    public static double proportionalConstantAngleDef = 0.02;
    public SpampleArm arm;
 
    public void Init(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime runtime, LinearOpMode opMode){
        super.init(hardwareMap, telemetry,0, 0, 0, opMode);
        spampleArm = new SpampleArm(hardwareMap, runtime);
}

    // overrides the autoMoveTo method in Robot.java to add in more year specific things.
    @Override
    public void autoMoveTo(double targetX, double targetY, double robotAngle, double targetCircle){
        telemetry.addData("distanceCircle", distanceCircle(targetX,targetY));
        telemetry.addData("active", opMode.opModeIsActive());
        while(distanceCircle(targetX, targetY) > targetCircle&&opMode.opModeIsActive()){
            odometry.updateOdometry();
            pathFollowing.setTargetPosition(targetX,targetY);
            anglePID.setTarget(robotAngle);
            //put all control things that teleop has in here
            anglePID.update(odometry.getRobotAngle());
            pathFollowing.followPath(odometry.getRobotX(),odometry.getRobotY(),odometry.getRobotAngle());
            driveTrain.setDrivePower(pathFollowing.getPowerF(), pathFollowing.getPowerS(), anglePID.getPower(), odometry.getRobotAngle());


            spampleArm.updateArm();

            telemetry.addData("X",odometry.getRobotX());
            telemetry.addData("Y", odometry.getRobotY());
            telemetry.addData("Angle", odometry.getRobotAngle());
            telemetry.update();
        }
    
        derivativeConstantAngle = derivativeConstantAngleDef;
        proportionalConstantAngle = proportionalConstantAngleDef;
    }
    @Override
    public void updateRobot(boolean holdPosition, boolean autoSpeedChange){
        if (autoSpeedChange) {
            if (spampleArm.currentArmState == SpampleArm.armState.idle) {
                driveTrain.setSpeedMultiplier(1);
            } else {
                driveTrain.setSpeedMultiplier(driveTrain.mediumSpeed);
            }
        }
        if (holdPosition){
            odometry.updateOdometry();
            anglePID.angleUpdate(odometry.getRobotAngle());
            pathFollowing.followPath(odometry.getRobotX(),odometry.getRobotY(),odometry.getRobotAngle());
            driveTrain.setDrivePower(pathFollowing.getPowerF(), pathFollowing.getPowerS(), anglePID.getPower(), odometry.getRobotAngle());
        }

        spampleArm.updateArm();
    }

    @Override
    public void chill(double seconds, boolean holdPosition){
        double startedTime = runtime.seconds();
        while (runtime.seconds() - startedTime < seconds && opMode.opModeIsActive()){
                updateRobot(holdPosition, false);
        }
    }
}
