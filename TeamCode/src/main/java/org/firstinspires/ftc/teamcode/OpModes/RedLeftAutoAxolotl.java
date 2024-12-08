package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Resources.Utlities;
import org.firstinspires.ftc.teamcode.Robot.Autonomous.AutonomousFunctions;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;


@Autonomous
public class RedLeftAutoAxolotl extends BaseAuto {

    Utlities utlities;

    //None of the values for RunToPoint Functions are correct
    @Override
    public void runOpMode() throws InterruptedException {

        utlities = new Utlities();

        initialize();

        robot.driveTrain.setFieldCentricDriving(false);

        robot.spampleArm.currentArmState = SpampleArm.armState.init;


        robot.driveTrain.setSpeedMultiplier(0.5);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

//        waitForStart();
        while (opModeInInit()){
            robot.updateRobot(false, false);
            telemetry.addData("currentArmState", robot.spampleArm.currentArmState);
            telemetry.update();
        }
        robot.odometry.overridePosition(-40,-63,0);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;


        robot.autoMoveTo(-62,-46,0,2);

        robot.spampleArm.currentArmState = SpampleArm.armState.highBasket;

        robot.chill(1.5, true);

        robot.driveTrain.setSpeedMultiplier(0.3);

        robot.autoMoveTo(-63,-56,0,0.5);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);

        robot.chill(0.5, true);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        robot.spampleArm.rotateTwistTo(0);

        robot.autoMoveTo(-62,-46,0,2);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;

        robot.chill(0.5, true);


        //Goes to the positon for hovering over sample
        robot.autoMoveTo(-48,-50,0,1);
        robot.spampleArm.currentArmState = SpampleArm.armState.grabSample;
        robot.spampleArm.rotateTwistTo(-90);

        //grabs the sample
        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
        robot.chill(2, true);
        robot.autoMoveTo(-49,-48.5,0,1);

        robot.spampleArm.currentArmState = SpampleArm.armState.grabSample2;

        robot.chill(1, true);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        robot.chill(1, true);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;


        robot.chill(1, true);

        robot.driveTrain.setSpeedMultiplier(0.4);

        robot.spampleArm.currentArmState = SpampleArm.armState.highBasket;

        robot.chill(1.5, true);

        robot.driveTrain.setSpeedMultiplier(0.3);

        robot.autoMoveTo(-63,-56,0,0.5);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);

        robot.chill(0.5, true);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        robot.spampleArm.rotateTwistTo(0);

        robot.autoMoveTo(-62,-46,0,2);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;

        robot.chill(0.7, true);

        robot.spampleArm.currentArmState = SpampleArm.armState.level1Assent;

        robot.autoMoveTo(-28,-3,-90,2);

        robot.chill(10, true);



    }
}
