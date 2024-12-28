package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.OpModes.BaseAuto.DConstantFast;
import static org.firstinspires.ftc.teamcode.OpModes.BaseAuto.DConstantSlow;
import static org.firstinspires.ftc.teamcode.OpModes.BaseAuto.IconstantFast;
import static org.firstinspires.ftc.teamcode.OpModes.BaseAuto.IconstantSlow;
import static org.firstinspires.ftc.teamcode.OpModes.BaseAuto.PconstantFast;
import static org.firstinspires.ftc.teamcode.OpModes.BaseAuto.PconstantSlow;
import static org.firstinspires.ftc.teamcode.OpModes.BaseAuto.speedMultplierFast;
import static org.firstinspires.ftc.teamcode.OpModes.BaseAuto.speedMultplierSlow;

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
        autoMoveTo(-40,-50,-90,2,0.2);
        autoMoveTo(-62,-46,0,2,0.2);

        driveTrain.setSpeedMultiplier(speedMultplierSlow);
        pathFollowing.xPID.updateConstants(PconstantSlow, IconstantSlow  , DConstantSlow);
        pathFollowing.yPID.updateConstants(PconstantSlow, IconstantSlow, DConstantSlow);

        spampleArm.currentArmState = SpampleArm.armState.highBasket;

        chill(1.5, true);

        //Drives directly above high basket
        autoMoveTo(-63,-56,0,0.5,0.2);

        //Drops sample
        spampleArm.setClawPosition(Claw.ClawPosition.open);

        chill(0.5, true);

        // Brings claw and twist to regular position
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        spampleArm.rotateTwistTo(0);

        // Drive slightly back from the high basket
        autoMoveTo(-62,-46,0,2,0.2);

        // Shoulder back to normal position
        spampleArm.currentArmState = SpampleArm.armState.idle;

        chill(0.5, true);

    }
    public void GrabRightSample(){
        driveTrain.setSpeedMultiplier(speedMultplierFast);
        pathFollowing.xPID.updateConstants(PconstantFast, IconstantFast  , DConstantFast);
        pathFollowing.yPID.updateConstants(PconstantFast, IconstantFast, DConstantFast);

        //Goes to the drive position for picking up sample
        autoMoveTo(-48,-50,0,1,0.2);

        driveTrain.setSpeedMultiplier(speedMultplierSlow);
        pathFollowing.xPID.updateConstants(PconstantSlow, IconstantSlow  , DConstantSlow);
        pathFollowing.yPID.updateConstants(PconstantSlow, IconstantSlow, DConstantSlow);


        //  goes to position for hovering over sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample;
        spampleArm.rotateTwistTo(-90);
        spampleArm.setClawPosition(Claw.ClawPosition.open);
        chill(2, true);
        autoMoveTo(-48,-47.5,0,0.5,0.2);


        //actually grabs the sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample2;
        chill(1, true);
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        chill(1, true);

        //back to original position
        spampleArm.currentArmState = SpampleArm.armState.idle;


        chill(1, true);
    }
    public void GrabMiddleSample(){
        driveTrain.setSpeedMultiplier(speedMultplierFast);
        pathFollowing.xPID.updateConstants(PconstantFast, IconstantFast  , DConstantFast);
        pathFollowing.yPID.updateConstants(PconstantFast, IconstantFast, DConstantFast);

        //Goes to the drive position for picking up sample
        autoMoveTo(-59,-37,0,2,0.2);

        driveTrain.setSpeedMultiplier(speedMultplierSlow);
        pathFollowing.xPID.updateConstants(PconstantSlow, IconstantSlow  , DConstantSlow);
        pathFollowing.yPID.updateConstants(PconstantSlow, IconstantSlow, DConstantSlow);


        //  goes to position for hovering over sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample;
        spampleArm.rotateTwistTo(-90);
        spampleArm.setClawPosition(Claw.ClawPosition.open);
        chill(2, true);
        autoMoveTo(-59,-34.5,0,0.5,0.2);


        //actually grabs the sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample2;
        chill(1, true);
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        chill(1, true);

        //back to original position
        spampleArm.currentArmState = SpampleArm.armState.idle;


        chill(1, true);
    }
    public void GrabLeftSample(){
        driveTrain.setSpeedMultiplier(speedMultplierFast);
        pathFollowing.xPID.updateConstants(PconstantFast, IconstantFast  , DConstantFast);
        pathFollowing.yPID.updateConstants(PconstantFast, IconstantFast, DConstantFast);

        //Goes to the drive position for picking up sample
        autoMoveTo(-68,-37,0,2,0.2);

        driveTrain.setSpeedMultiplier(speedMultplierSlow);
        pathFollowing.xPID.updateConstants(PconstantSlow, IconstantSlow  , DConstantSlow);
        pathFollowing.yPID.updateConstants(PconstantSlow, IconstantSlow, DConstantSlow);


        //  goes to position for hovering over sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample;
        spampleArm.rotateTwistTo(-90);
        spampleArm.setClawPosition(Claw.ClawPosition.open);
        chill(2, true);
        autoMoveTo(-68,-34.5,0,0.5,0.2);


        //actually grabs the sample
        spampleArm.currentArmState = SpampleArm.armState.grabSample2;
        chill(1, true);
        spampleArm.setClawPosition(Claw.ClawPosition.closed);
        chill(1, true);

        //back to original position
        spampleArm.currentArmState = SpampleArm.armState.idle;


        chill(1, true);
    }
}
