package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Resources.Utlities;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;
@Autonomous
public class CassadyAutoBestEver extends BaseAuto{

    Utlities utlities;
    @Override
    public void runOpMode() throws InterruptedException {

        utlities = new Utlities();

        initialize(true, true);

        robot.driveTrain.setFieldCentricDriving(false);

        robot.spampleArm.currentArmState = SpampleArm.armState.idle;

        setAutoSpeedFast();

        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        while (opModeInInit()) {
            robot.updateRobot(false, false, false);
            telemetry.addData("currentArmState", robot.spampleArm.currentArmState);
            telemetry.update();
        }
        //This is the starting position
        robot.autoMoveTo(0,99,90,2,7.5);
        robot.autoMoveTo(85,99,180, 2,7.5);
        robot.autoMoveTo(85,0,270,2,7.5);
        robot.autoMoveTo(0,0,0,2,7.5);
    }
}
