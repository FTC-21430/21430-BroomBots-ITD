package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Resources.Utlities;
import org.firstinspires.ftc.teamcode.Robot.Autonomous.AutonomousFunctions;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;


@Autonomous
public class RedRightAutoAxolotl extends BaseAuto {

    Utlities utlities;

    //None of the values for RunToPoint Functions are correct
    @Override
    public void runOpMode() throws InterruptedException {

        utlities = new Utlities();

        initialize(true);

        robot.driveTrain.setFieldCentricDriving(false);

        robot.spampleArm.currentArmState = SpampleArm.armState.init;

        robot.setRobotSpeed(Robot.Speed.FAST);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

//        waitForStart();
        while (opModeInInit()){
            robot.updateRobot(false, false);
            telemetry.addData("currentArmState", robot.spampleArm.currentArmState);
            telemetry.update();
        }
        robot.odometry.overridePosition(9,-63,90);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;

        robot.autoMoveTo(9,-40,0,1,3);

        robot.chill(2,true);

        robot.spampleArm.currentArmState = SpampleArm.armState.highChamber;

        setAutoSpeedSlow();

        robot.chill(1.4, true);

        robot.autoMoveTo(9,-33.4,0,1.5,3);

        robot.spampleArm.currentArmState = SpampleArm.armState.scoreHighChamber;

        robot.chill(1,false);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);

        robot.chill(1,true);

        robot.autoMoveTo(9,-50,0,2,3);

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        robot.spampleArm.currentArmState = SpampleArm.armState.idle;
//        setAutoSpeedFast();

//        robot.autoMoveTo(18,-36,0,1);

        robot.chill(1,true);

//        // push all of the red samples into observation zone
//        robot.autoMoveTo(34,-39,0,2);
//        robot.autoMoveTo(35,-11,0,2);
//        robot.autoMoveTo(46,-10,0,2);
//        robot.autoMoveTo(45,-56,0,2);
//        robot.autoMoveTo(47,-9,0,2);
//        robot.autoMoveTo(58,-10,0,2);
//        robot.autoMoveTo(60,-55,0,2);
//        robot.autoMoveTo(57,-10,0,2);
//        robot.autoMoveTo(64,-9,0,2);
//        robot.autoMoveTo(64,-53,0,2);
//
//        // ends in the observation zone


//        // from here, grab a specimen from human player
//        robot.autoMoveTo(47,-48,0,2);
//        robot.spampleArm.currentArmState = SpampleArm.armState.grabSpecimen;
//        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
//        setAutoSpeedSlow();
//        robot.chill(3, true);
//
//
//        robot.autoMoveTo(47,-63,0,1);
//
//        robot.chill(4,true);
//
//        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
//
//        robot.chill(1, true);
//
//        robot.autoMoveTo(47,-48,0,2);
//
//        robot.spampleArm.currentArmState = SpampleArm.armState.idle;
//
//        // now scoring a specimen using the same code from the start of this auto
//
//        robot.autoMoveTo(-3,-50,90,2);
//
//        robot.spampleArm.currentArmState = SpampleArm.armState.highChamber;
//
//        setAutoSpeedSlow();
//
//        robot.chill(1.4, true);
//
//        robot.autoMoveTo(-3,-40,0,2);
//
//        robot.spampleArm.currentArmState = SpampleArm.armState.scoreHighChamber;
//
//        robot.chill(1,false);
//
//        robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
//
//        robot.chill(1,true);
//
//        robot.autoMoveTo(-4,-50,0,2);
//
//        robot.spampleArm.currentArmState = SpampleArm.armState.idle;
//        setAutoSpeedFast();
//
//        // park in observation zone
//
        robot.autoMoveTo(30,-59,0,0.7,3);
        robot.chill(4,true);



    }
}
