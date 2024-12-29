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

//        robot.driveTrain.setSpeedMultiplier(speedMultplierFast);
//        robot.pathFollowing.xPID.updateConstants(PconstantFast, IconstantFast  , DConstantFast);
//        robot.pathFollowing.yPID.updateConstants(PconstantFast, IconstantFast, DConstantFast);
            setAutoSpeedSlow();

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        while (opModeInInit()){
            robot.updateRobot(false, false);
            telemetry.addData("currentArmState", robot.spampleArm.currentArmState);
            telemetry.update();
        }
        robot.odometry.overridePosition(-40,-63,-90);


       robot.ScoreSampleInHighBasket();

      robot.GrabRightSample();
//
      robot.ScoreSampleInHighBasket();

     //robot.GrabMiddleSample();

     //robot.ScoreSampleInHighBasket();
//
//       robot.GrabLeftSample();
//
//        robot.ScoreSampleInHighBasket();
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
