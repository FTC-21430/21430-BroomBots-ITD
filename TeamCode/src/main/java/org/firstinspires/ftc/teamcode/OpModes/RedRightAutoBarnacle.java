package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Resources.Utlities;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;
@Autonomous
public class RedRightAutoBarnacle extends BaseAuto{

    Utlities utlities;
    @Override
    public void runOpMode() throws InterruptedException {

        utlities = new Utlities();

        initialize(true);

        robot.driveTrain.setFieldCentricDriving(false);

        robot.spampleArm.currentArmState = SpampleArm.armState.init;

        setAutoSpeedFast();

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        while (opModeInInit()){
            robot.updateRobot(false, false);
            telemetry.addData("currentArmState", robot.spampleArm.currentArmState);
            telemetry.update();

        }
        //This is the starting position
        robot.odometry.overridePosition(8,-63,-90);
        robot.spampleArm.currentArmState = SpampleArm.armState.highChamber;

        robot.ScoreSpecimenHighChamber(0);

        robot.autoMoveTo(40, -48, 0,2,1);
        setAutoSpeedSlow();
        robot.spampleArm.currentArmState = SpampleArm.armState.test;
        robot.spampleArm.rotateTwistTo(-90);
        robot.spampleArm.rotateElbowTo(60);
        robot.autoMoveTo(49, -48, 0,2,1);


        robot.spampleArm.rotateShoulderTo(30);
        robot.spampleArm.extendTo(2);
        robot.chill(0.4,true);
        robot.spampleArm.currentArmState = SpampleArm.armState.test;
        robot.spampleArm.extendTo(1.6);
        robot.spampleArm.rotateShoulderTo(15.8);
        robot.spampleArm.rotateElbowTo(55);
        robot.chill(0.4,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        robot.chill(0.3,true);
        robot.spampleArm.currentArmState = SpampleArm.armState.dropSample;
        robot.autoMoveTo(58.5, -47,0,2,1);
        robot.chill(0.4,true);
        robot.spampleArm.currentArmState = SpampleArm.armState.test;
        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
        robot.chill(0.5,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        robot.spampleArm.rotateElbowTo(60);
        robot.spampleArm.rotateShoulderTo(34);
        robot.spampleArm.extendTo(2);

        robot.chill(0.5,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
        robot.spampleArm.currentArmState = SpampleArm.armState.test;
        robot.spampleArm.extendTo(1.6);
        robot.spampleArm.rotateShoulderTo(16.1);
        robot.spampleArm.rotateElbowTo(60);
        robot.chill(0.4,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        robot.chill(0.3,true);
        robot.spampleArm.currentArmState = SpampleArm.armState.dropSample;
        robot.chill(0.4,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);

        robot.setRobotSpeed(Robot.Speed.FAST);
        robot.autoMoveTo(37, -48.5, 0,2,0.3);
        robot.spampleArm.currentArmState = SpampleArm.armState.grabSpecimen;
        robot.chill(0.5,true);
        robot.autoMoveTo(37,-61.8,0,2,0.3);
        robot.chill(0.5,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        robot.chill(0.3,true);
        robot.setRobotSpeed(Robot.Speed.FAST);
        robot.ScoreSpecimenHighChamber(1.5);


        robot.autoMoveTo(37, -48, 0,2,0.3);
        robot.spampleArm.currentArmState = SpampleArm.armState.grabSpecimen;
        robot.chill(0.6,true);
        robot.autoMoveTo(37,-61.8,0,2,0.3);
        robot.chill(0.5,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        robot.chill(0.3,true);
        robot.ScoreSpecimenHighChamber(3);
        robot.setRobotSpeed(Robot.Speed.FAST);


        robot.autoMoveTo(37, -48, 0,2,0.3);
        robot.spampleArm.currentArmState = SpampleArm.armState.grabSpecimen;
        robot.chill(0.6,true);
        robot.autoMoveTo(37,-61.8,0,2,0.3);
        robot.chill(0.5,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        robot.chill(0.3,true);
        robot.ScoreSpecimenHighChamber(4.5,1);
        robot.setRobotSpeed(Robot.Speed.FAST);

        robot.autoMoveTo(40, -52, 0,0.5,1);
    }
}
