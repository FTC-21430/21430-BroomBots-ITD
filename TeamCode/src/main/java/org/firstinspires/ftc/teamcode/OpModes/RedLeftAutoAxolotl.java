package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.Robot.Autonomous.AutonomousFunctions;

public class RedLeftAutoAxolotl extends BaseAuto {
    
    // TODO Update all of these runto point functions to be the updated AutoMoveTo function in robot.java
    
    
    //None of the values for RunToPoint Functions are correct
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousFunctions.WaitFunction();
        RunToPoint(48, 10, 0.2);
        robot.arm.highChamber();
        RunToPoint(24, 15,0.1);

        for (int i = 0; i<3; i++)
        {
            robot.arm.grabSpample();
            RunToPoint(-12,-34,0.3);
            robot.arm.highBasket();
            RunToPoint(12,34, 0.3);

        }
        RunToPoint(24, 15, 0.4);
        //TODO Add Climb Level 1 Assent funciton
    }
}
