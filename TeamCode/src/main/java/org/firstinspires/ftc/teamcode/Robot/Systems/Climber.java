package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Base64;

// This class has the functions for the climbers
public class Climber {
    // These are the two motors that control the climber and they run in parallel
    DcMotor leftClimberMotor;
    DcMotor rightClimberMotor;

    public Climber(){
        startingPosition();
    }

    //THIS IS A PLACEHOLDER. More testing is needed.
    final private double extended = 9.0;

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
    public void level1Ascent() {
            leftClimberMotor.setTargetPosition((int) (extended*ticksPerInches));
            rightClimberMotor.setTargetPosition((int) (extended*ticksPerInches));
    }

    // The motors pull the robot up to the bar
    public void level2Ascent(){
            leftClimberMotor.setTargetPosition(0);
            rightClimberMotor.setTargetPosition(0);
    }

    // The motors extend to the highest bar and then pull it up
    public void level3Ascent(){
            leftClimberMotor.setTargetPosition((int) (extended*ticksPerInches));
            rightClimberMotor.setTargetPosition((int) (extended*ticksPerInches));
            leftClimberMotor.setTargetPosition(0);
            rightClimberMotor.setTargetPosition(0);
    }
}
