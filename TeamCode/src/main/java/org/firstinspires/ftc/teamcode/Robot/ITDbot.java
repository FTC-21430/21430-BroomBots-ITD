package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

public class ITDbot extends Robot {
    //TODO tune these numbers for ITDbot, need to be updated for Into the deep
    public static double derivativeConstantAngleDef = 0.0015;
    public static double proportionalConstantAngleDef = 0.02;
    public SpampleArm arm;
 
    public void Init(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime runtime, LinearOpMode opMode,boolean reset, boolean isAuto){
        super.init(hardwareMap, telemetry,0, 0, 0, opMode,reset, isAuto);
        spampleArm = new SpampleArm(hardwareMap, runtime,reset, isAuto);
}

    // overrides the autoMoveTo method in Robot.java to add in more year specific things.
    @Override
    public void autoMoveTo(double targetX, double targetY, double robotAngle, double targetCircle,double Timeout ){
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

            //TODO, if wanting to use auto move to during teleop, fix the tuning issue between teleop and auto
            spampleArm.updateArm(true);

            telemetry.addData("X",odometry.getRobotX());
            telemetry.addData("Y", odometry.getRobotY());
            telemetry.addData("Angle", odometry.getRobotAngle());
            telemetry.update();
        }
    
        derivativeConstantAngle = derivativeConstantAngleDef;
        proportionalConstantAngle = proportionalConstantAngleDef;

    }
    @Override
    public void updateRobot(boolean holdPosition, boolean autoSpeedChange, boolean isAuto){
        if (autoSpeedChange) {
            if (spampleArm.currentArmState == SpampleArm.armState.idle) {
                driveTrain.setSpeedMultiplier(1);
            } else {
                driveTrain.setSpeedMultiplier(driveTrain.mediumSpeed);
            }
        }
        if (holdPosition){
            odometry.updateOdometry();
            anglePID.update(odometry.getRobotAngle());
            pathFollowing.followPath(odometry.getRobotX(),odometry.getRobotY(),odometry.getRobotAngle());
            driveTrain.setDrivePower(pathFollowing.getPowerF(), pathFollowing.getPowerS(), anglePID.getPower(), odometry.getRobotAngle());
        }

        spampleArm.updateArm(isAuto);
    }

    @Override
    public void chill(double seconds, boolean holdPosition){
        double startedTime = runtime.seconds();
        while (runtime.seconds() - startedTime < seconds && opMode.opModeIsActive()){
                updateRobot(holdPosition, false, true);
        }
    }

    public void ScoreSampleInHighBasket(){
        // Drives over to the high basket
        setRobotSpeed(Robot.Speed.SLOW);
        spampleArm.currentArmState = SpampleArm.armState.highBasket;
        autoMoveTo(-54,-54,-40,2,3);
        chill(.7, true);

        // Drives directly above high basket
        driveTrain.setSpeedMultiplier(0.35);
        autoMoveTo(-57.5,-56.5,-40,1.5,2);
        chill(0.2, true);
        driveTrain.setSpeedMultiplier(0.4);

        //Drops sample
        spampleArm.setClawPosition(Claw.ClawPosition.open);
        chill(0.45, true);

        // Brings claw and twist to regular position
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        spampleArm.rotateTwistTo(0);

        // Drive slightly back from the high basket
        autoMoveTo(-55,-46,0,2,0.5);
        spampleArm.currentArmState = SpampleArm.armState.test;

        // Shoulder back to normal position
        spampleArm.rotateTwistTo(0);
        spampleArm.rotateElbowTo(65);
        spampleArm.extendTo(2);
        spampleArm.rotateShoulderTo(90);
        chill(0.3, true);
    }

    public void GrabRightSample(){
       // Prepares (kind of like initializing)
        setRobotSpeed(Robot.Speed.FAST);
        spampleArm.rotateShoulderTo(69);
        chill(0.4,true);

        // Goes to position for hovering over sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample;
        spampleArm.rotateTwistTo(-90);
        spampleArm.setClawPosition(Claw.ClawPosition.open);
        spampleArm.rotateElbowTo(65);
       
        // Goes to the drive position for picking up sample
        autoMoveTo(-48,-50.5,0,1,0.2);
        setRobotSpeed(Robot.Speed.SLOW);
//        spampleArm.rotateShoulderTo(32);
        chill(0.6,true);

        //  Goes to position for hovering over sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample2;
        chill(.5, true);

        // Actually grabs the sample
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        chill(0.3, true);

        // Back to original position
        spampleArm.currentArmState = SpampleArm.armState.idle;
        chill(0.4, true);
    }
    public void GrabMiddleSample(){
        // Prepares (kind of like initialization)
        spampleArm.rotateElbowTo(65);
        spampleArm.rotateTwistTo(-90);
        spampleArm.setClawPosition(Claw.ClawPosition.open);
        setRobotSpeed(Robot.Speed.FAST);
        spampleArm.rotateShoulderTo(50);
        spampleArm.rotateElbowTo(65);

        // Goes to the drive position for picking up sample
        autoMoveTo(-57.4,-50.5,0,1,0.2);
        setRobotSpeed(Robot.Speed.SLOW);
        chill(0.4,true);

        // Goes to position for hovering over sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample2;
        chill(0.7, true);

//        // Actually grabs the sample
//        spampleArm.currentArmState = SpampleArm.armState.grabSample2;
        chill(0.3, true);
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        chill(0.3, true);

        // Returns to original position
        spampleArm.currentArmState = SpampleArm.armState.idle;
        chill(0.4, true);
    }
    public void GrabLeftSample() {
        // Prepares (kind of like initialization)
        spampleArm.rotateElbowTo(65);
        spampleArm.rotateTwistTo(50);
        spampleArm.setClawPosition(Claw.ClawPosition.open);
        autoMoveTo(-54.5,-42.2,30,2,0.2);
        setRobotSpeed(Robot.Speed.FAST);

        // Goes to the drive position for picking up sample
        autoMoveTo(-54.5,-44,30,1,0.4);
        setRobotSpeed(Robot.Speed.SLOW);
        chill(0.3,true);

        // Goes to position for hovering over sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample;
        chill(0.4, true);
        autoMoveTo(-57,-46.4,30,1,0.2);
        chill(0.4, true);

        // Actually grabs the sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample2;
        chill(0.3, true);
        spampleArm.currentArmState = SpampleArm.armState.test;
        spampleArm.rotateShoulderTo(15.5);
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        chill(0.3, true);

        // Return to original position
        spampleArm.currentArmState = SpampleArm.armState.idle;
        autoMoveTo(-51,-47,0,1,0.2);
    }
    public void ScoreSpecimenHighChamber(double offsetX){
        ScoreSpecimenHighChamber(offsetX,0);
    }
    public void ScoreSpecimenHighChamber(double offsetX,double offsetY){
        setRobotSpeed(Speed.FAST);
        spampleArm.rotateElbowTo(76.5);
        //goes to drive position for scoring specimen
        spampleArm.currentArmState = SpampleArm.armState.highChamber;
        autoMoveTo(0 + offsetX,-52,0,2,0.2);
        chill(0.5,true);
        autoMoveTo(0+offsetX,-34.5+offsetY,0,2,0.2);
        chill(0.15,true);
        spampleArm.setClawPosition(Claw.ClawPosition.open);
        chill(0.3,true);
        autoMoveTo(0,-42,0,2,0.2);
        spampleArm.currentArmState= SpampleArm.armState.idle;

    }
}
