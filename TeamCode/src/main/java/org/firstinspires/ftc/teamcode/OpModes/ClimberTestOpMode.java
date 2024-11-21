package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.Systems.Climber;

public class ClimberTestOpMode extends BaseTeleOp{
    @Override
    public void runOpMode() throws InterruptedException {
        Climber climber = new Climber(hardwareMap);
        Gamepad gamepad2 = new Gamepad();
//        if(gamepad2.dpad_left){
//            climber.level1Ascent();
//        }
//        if(gamepad2.dpad_up){
//            climber.level2Ascent();
//        }
//        if(gamepad2.dpad_right){
//            climber.level3Ascent();
//        }
    }
}
