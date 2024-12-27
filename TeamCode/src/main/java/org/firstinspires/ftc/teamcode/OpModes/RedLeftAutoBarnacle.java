package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Resources.Utlities;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@Autonomous
public class RedLeftAutoBarnacle extends BaseAuto {
    Utlities utlities;

    //None of the values for RunToPoint Functions are correct
    @Override
    public void runOpMode() throws InterruptedException {

        utlities = new Utlities();

        initialize();

        robot.driveTrain.setFieldCentricDriving(false);

        robot.spampleArm.currentArmState = SpampleArm.armState.init;


        robot.driveTrain.setSpeedMultiplier(speedMultplierFast);
        robot.pathFollowing.xPID.updateConstants(PconstantFast, IconstantFast  , DConstantFast);
        robot.pathFollowing.yPID.updateConstants(PconstantFast, IconstantFast, DConstantFast);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        while (opModeInInit()){
            robot.updateRobot(false, false);
            telemetry.addData("currentArmState", robot.spampleArm.currentArmState);
            telemetry.update();
        }
        robot.odometry.overridePosition(-40,-63,-90);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;

        robot.autoMoveTo(-40,-50,-90,2);
        robot.autoMoveTo(-62,-46,0,2);

        robot.driveTrain.setSpeedMultiplier(speedMultplierSlow);
        robot.pathFollowing.xPID.updateConstants(PconstantSlow, IconstantSlow  , DConstantSlow);
        robot.pathFollowing.yPID.updateConstants(PconstantSlow, IconstantSlow, DConstantSlow);

        robot.spampleArm.currentArmState = SpampleArm.armState.highBasket;

        robot.chill(1.5, true);

        robot.autoMoveTo(-63,-56,0,0.5);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);

        robot.chill(0.5, true);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        robot.spampleArm.rotateTwistTo(0);

        robot.autoMoveTo(-62,-46,0,2);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;

        robot.chill(0.5, true);


        robot.driveTrain.setSpeedMultiplier(speedMultplierFast);
        robot.pathFollowing.xPID.updateConstants(PconstantFast, IconstantFast  , DConstantFast);
        robot.pathFollowing.yPID.updateConstants(PconstantFast, IconstantFast, DConstantFast);

        //Goes to the positon for hovering over sample
        robot.autoMoveTo(-48,-50,0,1);

        robot.driveTrain.setSpeedMultiplier(speedMultplierSlow);
        robot.pathFollowing.xPID.updateConstants(PconstantSlow, IconstantSlow  , DConstantSlow);
        robot.pathFollowing.yPID.updateConstants(PconstantSlow, IconstantSlow, DConstantSlow);

        robot.spampleArm.currentArmState = SpampleArm.armState.grabSample;
        robot.spampleArm.rotateTwistTo(-90);

        //grabs the sample
        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
        robot.chill(2, true);
        robot.autoMoveTo(-48,-47.5,0,0.5);

        robot.spampleArm.currentArmState = SpampleArm.armState.grabSample2;

        robot.chill(1, true);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        robot.chill(1, true);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;


        robot.chill(1, true);

        robot.spampleArm.currentArmState = SpampleArm.armState.highBasket;

        robot.chill(1.5, true);

        robot.autoMoveTo(-63,-56,0,0.5);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);

        robot.chill(0.5, true);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        robot.spampleArm.rotateTwistTo(0);

        robot.autoMoveTo(-62,-46,0,2);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;

        robot.chill(0.7, true);






        robot.driveTrain.setSpeedMultiplier(speedMultplierFast);
        robot.pathFollowing.xPID.updateConstants(PconstantFast, IconstantFast  , DConstantFast);
        robot.pathFollowing.yPID.updateConstants(PconstantFast, IconstantFast, DConstantFast);

        //Goes to the positon for hovering over sample
        robot.autoMoveTo(-58,-35,0,2);

        robot.driveTrain.setSpeedMultiplier(speedMultplierSlow);
        robot.pathFollowing.xPID.updateConstants(PconstantSlow, IconstantSlow  , DConstantSlow);
        robot.pathFollowing.yPID.updateConstants(PconstantSlow, IconstantSlow, DConstantSlow);

        robot.spampleArm.currentArmState = SpampleArm.armState.grabSample;
        robot.spampleArm.rotateTwistTo(-90);

        //grabs the sample
        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
        robot.chill(2, true);
        robot.autoMoveTo(-48,-47.5,0,0.5);

        robot.spampleArm.currentArmState = SpampleArm.armState.grabSample2;

        robot.chill(1, true);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        robot.chill(1, true);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;


        robot.chill(1, true);

        robot.spampleArm.currentArmState = SpampleArm.armState.highBasket;

        robot.chill(1.5, true);

        robot.autoMoveTo(-63,-56,0,0.5);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);

        robot.chill(0.5, true);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        robot.spampleArm.rotateTwistTo(0);

        robot.autoMoveTo(-62,-46,0,2);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;

        robot.chill(0.7, true);







//        robot.driveTrain.setSpeedMultiplier(speedMultplierFast);
//        robot.pathFollowing.xPID.updateConstants(PconstantFast, IconstantFast  , DConstantFast);
//        robot.pathFollowing.yPID.updateConstants(PconstantFast, IconstantFast, DConstantFast);
//        robot.autoMoveTo(-36,-12,-90,2);
//
//        robot.spampleArm.currentArmState = SpampleArm.armState.level1Assent;
//
//        robot.autoMoveTo(-26.5,-12,-90,2);
    }

}
