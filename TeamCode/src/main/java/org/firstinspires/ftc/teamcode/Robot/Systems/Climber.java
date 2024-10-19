package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// This class has the functions for the climbers
public class Climber {
    // These are the two motors that control the climber and they run in parallel
    DcMotor leftClimberMotor;
    DcMotor rightClimberMotor;
    Gamepad gamepad1;
    Gamepad gamepad2;
    // This function is always the starting position of the climber at 0,0

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
                leftClimberMotor.setTargetPosition(1);
                rightClimberMotor.setTargetPosition(1);
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
            leftClimberMotor.setTargetPosition(1);
            rightClimberMotor.setTargetPosition(1);
            leftClimberMotor.setTargetPosition(0);
            rightClimberMotor.setTargetPosition(0);
        }

    }
}
