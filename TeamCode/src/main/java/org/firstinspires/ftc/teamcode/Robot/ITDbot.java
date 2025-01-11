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
 
    public void Init(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime runtime, LinearOpMode opMode){
        super.init(hardwareMap, telemetry,0, 0, 0, opMode);
        spampleArm = new SpampleArm(hardwareMap, runtime);
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
            anglePID.update(odometry.getRobotAngle());
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
    public void ScoreSampleInHighBasket(){
        // Drives over to the high basket
        setRobotSpeed(Robot.Speed.SLOW);
        spampleArm.currentArmState = SpampleArm.armState.highBasket;
        autoMoveTo(-55,-54,-40,2,3);





        chill(.3, true);

        //Drives directly above high basket
        driveTrain.setSpeedMultiplier(0.35);
        autoMoveTo(-57.5,-57.5,-40,1.5,2);
        chill(0.5, true);
        driveTrain.setSpeedMultiplier(0.4);


        //Drops sample
        spampleArm.setClawPosition(Claw.ClawPosition.open);

        chill(0.6, true);

        // Brings claw and twist to regular position
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        spampleArm.rotateTwistTo(0);

        // Drive slightly back from the high basket
        autoMoveTo(-55,-46,0,2,0.5);

        // Shoulder back to normal position
        spampleArm.currentArmState = SpampleArm.armState.idle;

        chill(0.5, true);

    }
    public void GrabRightSample(){
        setRobotSpeed(Robot.Speed.FAST);
        spampleArm.rotateShoulderTo(64);
        chill(0.4,true);
        //  goes to position for hovering over sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample;
        spampleArm.rotateTwistTo(-90);
        spampleArm.setClawPosition(Claw.ClawPosition.open);

        //Goes to the drive position for picking up sample
        autoMoveTo(-48,-47,0,1,0.2);

        setRobotSpeed(Robot.Speed.FAST);

        chill(0.5, true);
        autoMoveTo(-47.2,-46.2,0,1,0.2);


        //actually grabs the sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample2;
        chill(0.3, true);
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        chill(0.6, true);

        //back to original position
        spampleArm.currentArmState = SpampleArm.armState.idle;


        chill(0.4, true);
    }
    public void GrabMiddleSample(){
        setRobotSpeed(Robot.Speed.FAST);
        spampleArm.rotateShoulderTo(64);
        chill(0.4,true);
        //  goes to position for hovering over sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample;
        spampleArm.rotateTwistTo(-90);
        spampleArm.setClawPosition(Claw.ClawPosition.open);

        //Goes to the drive position for picking up sample
        autoMoveTo(-55.7,-46.5,0,1,0.2);

        setRobotSpeed(Robot.Speed.SLOW);

        chill(0.5, true);
        autoMoveTo(-55.7,-47,0,1,0.2);


        //actually grabs the sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample2;
        chill(0.3, true);
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        chill(0.6, true);

        //back to original position
        spampleArm.currentArmState = SpampleArm.armState.idle;


        chill(0.4, true);
    }
    public void GrabLeftSample() {
        autoMoveTo(-54.5,-45,30,2,0.2);
        setRobotSpeed(Robot.Speed.FAST);


        spampleArm.rotateShoulderTo(64);
        chill(1,true);
        //  goes to position for hovering over sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample;
        spampleArm.rotateTwistTo(-90);
        spampleArm.setClawPosition(Claw.ClawPosition.open);

        //Goes to the drive position for picking up sample
        autoMoveTo(-56.5,-45,30,1,0.2);

        setRobotSpeed(Robot.Speed.SLOW);

        chill(0.5, true);

        autoMoveTo(-57,-44.5,30,1,0.2);


        //actually grabs the sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample2;
        chill(0.3, true);
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        chill(0.4, true);

        //back to original position
        spampleArm.currentArmState = SpampleArm.armState.idle;

        autoMoveTo(-57.5,-46,0,1,0.2);
    }
}
