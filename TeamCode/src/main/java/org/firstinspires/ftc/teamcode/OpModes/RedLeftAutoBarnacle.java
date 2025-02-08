package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Resources.Utlities;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@Autonomous
public class RedLeftAutoBarnacle extends BaseAuto {
    Utlities utlities;
    @Override
    public void runOpMode() throws InterruptedException {

        utlities = new Utlities();

        initialize(true, true);

        robot.driveTrain.setFieldCentricDriving(false);

        robot.spampleArm.currentArmState = SpampleArm.armState.init;

        setAutoSpeedSlow();

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        while (opModeInInit()){
            robot.updateRobot(false, false, true);
            telemetry.addData("currentArmState", robot.spampleArm.currentArmState);
            telemetry.update();
        }

        // Sets starting position
        robot.odometry.overridePosition(-40,-63,-90);

        robot.spampleArm.currentArmState = SpampleArm.armState.test;
        robot.spampleArm.extendTo(1);

        robot.autoMoveTo(-42,-50,-40,2,3);

       robot.ScoreSampleInHighBasket();
       robot.chill(0.4,true);
       robot.spampleArm.calibrateExtension();
       robot.chill(0.6,true);


      robot.GrabRightSample();

      robot.ScoreSampleInHighBasket();

      robot.GrabMiddleSample();

     robot.ScoreSampleInHighBasket();

        robot.GrabLeftSample();

        robot.ScoreSampleInHighBasket();


        robot.setRobotSpeed(Robot.Speed.FAST);


        robot.spampleArm.calibrateExtension();
        robot.chill(0.4, true);
        robot.autoMoveTo(-38,-12, -90, 2, 3);

        robot.autoMoveTo(-26,-12, -90, 4, 3);
        robot.spampleArm.currentArmState = SpampleArm.armState.level1Assent;
        robot.chill(3,false);
    }


}
