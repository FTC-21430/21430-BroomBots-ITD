package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ITDbot;
// Creates robot object. All OpModes inherit this.
abstract public class GeneralOpMode extends LinearOpMode {
    public ITDbot robot;
    
    public void initialize() {
      // robot = new ITDbot();
        //robot.Init(hardwareMap,telemetry);
        robot.setIsOpModeActive(opModeIsActive());
    }
}
