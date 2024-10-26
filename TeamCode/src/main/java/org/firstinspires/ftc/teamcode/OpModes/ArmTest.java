package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@TeleOp
public class ArmTest extends LinearOpMode {
    SpampleArm spampleArm;
    @Override
    public void runOpMode() throws InterruptedException {
        spampleArm = new SpampleArm(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
           
            if (gamepad2.a) {
                spampleArm.rotateWristTo(90);
            }
            if (gamepad2.b) {
                spampleArm.rotateWristTo(0);
            }

            if (gamepad2.right_bumper) {
                spampleArm.extendTo(23);
            }
            if (gamepad2.left_bumper) {
                spampleArm.extendTo(0);
            }
            if (gamepad2.dpad_up) {
                spampleArm.setClawPosition(Claw.ClawPosition.open);
            }
            if (gamepad2.dpad_down) {
                spampleArm.setClawPosition(Claw.ClawPosition.closed);
            }
            if (gamepad2.right_trigger >= 0.5F){
                spampleArm.rotateShoulderTo(180);
            }
            else{
                spampleArm.rotateShoulderTo(0);
            }
            
            
            telemetry.update();
            
            
        }
    }
}
