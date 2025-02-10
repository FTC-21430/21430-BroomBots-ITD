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

        initialize(true, true);

        robot.driveTrain.setFieldCentricDriving(false);

        robot.spampleArm.currentArmState = SpampleArm.armState.init;

        setAutoSpeedFast();

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        while (opModeInInit()){
            robot.updateRobot(false, false, false);
            telemetry.addData("currentArmState", robot.spampleArm.currentArmState);
            telemetry.update();

        }
        //This is the starting position
        robot.odometry.overridePosition(8,-63,-90);
        robot.spampleArm.currentArmState = SpampleArm.armState.test;
        robot.spampleArm.extendTo(2);
        robot.chill(0.4,true);
        robot.spampleArm.currentArmState = SpampleArm.armState.highChamber;


        robot.setRobotSpeed(Robot.Speed.FAST);
        robot.spampleArm.rotateElbowTo(79);
        //goes to drive position for scoring specimen
        robot.spampleArm.currentArmState = SpampleArm.armState.highChamber;
        robot.autoMoveTo(0,-52,0,2,0.2);
        robot.chill(0.5,true);
        robot.autoMoveTo(0,-34.5,0,2,0.2);
        robot.chill(0.15,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
        robot.chill(0.3,true);
        robot.autoMoveTo(0,-42,0,2,0.2);
        robot.spampleArm.currentArmState= SpampleArm.armState.idle;


        robot.autoMoveTo(40, -48, 0,2,1);
        setAutoSpeedSlow();
        robot.spampleArm.currentArmState = SpampleArm.armState.test;
        robot.spampleArm.rotateTwistTo(-90);
        robot.spampleArm.rotateElbowTo(60);
        robot.autoMoveTo(49.7, -48, 0,2,1);

        robot.spampleArm.rotateShoulderTo(30);
        robot.spampleArm.extendTo(2);
        robot.chill(0.4,true);
        robot.spampleArm.currentArmState = SpampleArm.armState.test;
        robot.spampleArm.extendTo(1.6);
        robot.spampleArm.rotateShoulderTo(17);
        robot.spampleArm.rotateElbowTo(55);
        robot.chill(0.4,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        robot.chill(0.3,true);
        robot.spampleArm.currentArmState = SpampleArm.armState.dropSample;
        robot.autoMoveTo(58.5, -47.2,0,2,1);
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
        robot.spampleArm.rotateShoulderTo(18.5);
        robot.spampleArm.rotateElbowTo(60);
        robot.chill(0.4,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        robot.chill(0.3,true);
        robot.spampleArm.currentArmState = SpampleArm.armState.dropSample;
        robot.chill(0.4,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
        robot.chill(0.5,true);
        robot.setRobotSpeed(Robot.Speed.FAST);
        robot.autoMoveTo(37, -48, 0,2,0.3);
        robot.spampleArm.currentArmState = SpampleArm.armState.grabSpecimen;
        robot.chill(0.5,true);
        robot.autoMoveTo(37,-61.8,0,2,0.3);
        robot.chill(0.5,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        robot.chill(0.3,true);
        robot.setRobotSpeed(Robot.Speed.FAST);

        robot.ScoreSpecimenHighChamber(2);


        robot.autoMoveTo(34, -48, 0,2,0.3);
        robot.spampleArm.currentArmState = SpampleArm.armState.grabSpecimen;
        robot.chill(0.6,true);
        robot.autoMoveTo(34,-61.8,0,2,0.3);
        robot.chill(0.5,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        robot.chill(0.3,true);
        robot.ScoreSpecimenHighChamber(4);
        robot.setRobotSpeed(Robot.Speed.FAST);


        robot.autoMoveTo(34, -48, 0,2,0.3);
        robot.spampleArm.currentArmState = SpampleArm.armState.grabSpecimen;
        robot.chill(0.6,true);
        robot.autoMoveTo(34,-61.8,0,2,0.3);
        robot.chill(0.5,true);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        robot.chill(0.3,true);
        robot.ScoreSpecimenHighChamber(6,1);
        robot.setRobotSpeed(Robot.Speed.FAST);
        robot.autoMoveTo(40, -52, 0,7,1);
        robot.setRobotSpeed(Robot.Speed.SLOW);
        robot.autoMoveTo(40, -52, 0,0.5,1);
    }
}
