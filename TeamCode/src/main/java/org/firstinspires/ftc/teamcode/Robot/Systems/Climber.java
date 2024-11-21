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
    // These are the two motors that control the climber and they run in parallel
    private Telemetry telemetry;
    public Climber(HardwareMap hardwareMap){
        rightClimberMotor = hardwareMap.get (DcMotor.class, "rightClimberMotor");
        leftClimberMotor = hardwareMap.get(DcMotor.class, "leftClimberMotor");

        leftClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        startingPosition();

        leftClimberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightClimberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftClimberMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightClimberMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    DcMotor leftClimberMotor;
    DcMotor rightClimberMotor;


    //THIS IS A PLACEHOLDER. More testing is needed.
    final private double extensionPosition = 1.0;

    // This function is always the starting position of the climber at 0,0

    //THESE NUMBERS ARE PLACEHOLDERS! 25.4 is mm per inch|2000 is ticks per revolution|48*Math.PI is
    // mm per revolution
    private double ticksPerInches = 25.4*2000/(48 * Math.PI);

    // The motors go to their lowest extension
    public void startingPosition(){
        leftClimberMotor.setTargetPosition(0);
        rightClimberMotor.setTargetPosition(0);
    }

    // The motors extend to the height required to climb
    public void climberAssent(){
        leftClimberMotor.setTargetPosition((int)(extensionPosition*ticksPerInches));
        rightClimberMotor.setTargetPosition((int)(extensionPosition*ticksPerInches));
    }
}
