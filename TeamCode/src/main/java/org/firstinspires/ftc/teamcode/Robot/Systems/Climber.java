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
    Gamepad gamepad2;

    //THIS IS A PLACEHOLDER. More testing is needed.
    final private double extended = 9.0;

    // This function is always the starting position of the climber at 0,0

    //THESE NUMBERS ARE PLACEHOLDERS! 25.4 is mm per inch|2000 is ticks per revolution|48*Math.PI is
    // mm per revolution
    private double ticksPerInches = 25.4*2000/(48 * Math.PI);

    // The motors go to their lowest extension
    private void startingPosition(){
        if (gamepad2.dpad_down){
        leftClimberMotor.setTargetPosition(0);
        rightClimberMotor.setTargetPosition(0);
        }
    }

    // The motors extend to the height required to climb
    private void level1Ascent() {
        if (gamepad2.dpad_right) {
            leftClimberMotor.setTargetPosition((int) (extended*ticksPerInches));
            rightClimberMotor.setTargetPosition((int) (extended*ticksPerInches));
        }
    }

    // The motors pull the robot up to the bar
    private void level2Ascent(){
        if (gamepad2.dpad_left){
            leftClimberMotor.setTargetPosition(0);
            rightClimberMotor.setTargetPosition(0);
        }
    }

    // The motors extend to the highest bar and then pull it up
    private void level3Ascent(){
        if (gamepad2.dpad_up){
            leftClimberMotor.setTargetPosition((int) (extended*ticksPerInches));
            rightClimberMotor.setTargetPosition((int) (extended*ticksPerInches));
            leftClimberMotor.setTargetPosition(0);
            rightClimberMotor.setTargetPosition(0);
        }
    }
}
