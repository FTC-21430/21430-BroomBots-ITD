package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.ITDbot;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;

// Creates robot object. All OpModes inherit this.
abstract public class GeneralOpMode extends LinearOpMode {
    public ITDbot robot;

    public ElapsedTime runtime = new ElapsedTime();
    
    
    // variables for state machine logic

    

    

    // normal functions
    
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new ITDbot();
        robot.Init(hardwareMap,telemetry,runtime, this);
        
    }
    

    // Arm state machine logic is here because all Op-modes need to use it.
    

    
}
