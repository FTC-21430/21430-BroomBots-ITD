package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Base64;

// This class has the functions for the climbers
public class Climber {

    // This function is always the starting position of the climber at 0,0

    //THESE NUMBERS ARE PLACEHOLDERS! 25.4 is mm per inch|2000 is ticks per revolution|48*Math.PI is
    // mm per revolution
    private double ticksPerInches = 364.9/4.469;

    private double inchesPerTicks = 4.469/364.9;

    private double minExtension = 0.0; //inches

    private double maxExtension = 12.75;



    DcMotor leftClimberMotor;
    DcMotor rightClimberMotor;
    // These are the two motors that control the climber and they run in parallel
    private Telemetry telemetry;
    public Climber(HardwareMap hardwareMap, Telemetry telemetry){
        rightClimberMotor = hardwareMap.get (DcMotor.class, "rightClimberMotor");
        leftClimberMotor = hardwareMap.get(DcMotor.class, "leftClimberMotor");

        leftClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftClimberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightClimberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftClimberMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightClimberMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftClimberMotor.setPower(1);
        rightClimberMotor.setPower(1);

        startingPosition();

        this.telemetry = telemetry;
    }



    // The motors go to their lowest extension
    public void startingPosition(){
        leftClimberMotor.setTargetPosition(0);
        rightClimberMotor.setTargetPosition(0);
    }


    public void extendTo(double inches){

        if (inches > maxExtension) {
            inches = maxExtension;
        }
        if (inches < minExtension) {
            inches = minExtension;
        }

        leftClimberMotor.setTargetPosition((int)(inches * ticksPerInches));
        rightClimberMotor.setTargetPosition((int)(inches * ticksPerInches));

    }

    public double getCurrentExtension(){
        return (leftClimberMotor.getCurrentPosition() + rightClimberMotor.getCurrentPosition())/2 * inchesPerTicks;
    }
}