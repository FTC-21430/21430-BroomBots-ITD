package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.OpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.Robot.Autonomous.AutonomousFunctions;

// TODO Update all of these runto point functions to be the updated AutoMoveTo function in robot.java


public class RedRightAutoAxolotl extends BaseAuto {

    //None of the values for RunToPoint Functions are correct
    @Override
    public void runOpMode() throws InterruptedException {
        RunToPoint(48, 12, 0.3);
        robot.arm.highChamber();
        RunToPoint(48,12, 0.4);
        robot.arm.grabSpample();
        RunToPoint(48, 8, 0.3);
        robot.arm.dropOff();
        RunToPoint(50, 12, 0.3);
        robot.arm.grabSpample();
        RunToPoint(48,8,0.3);
        robot.arm.dropOff();
        //add grab spicement
        RunToPoint(42,24, 0.2);
        robot.arm.highChamber();
        RunToPoint(28,-10,0.2);
    }
}