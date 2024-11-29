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
            
            
            if(gamepad1.a){
                spampleArm.rotateShoulderTo(90);
            }
            if (gamepad1.b){
                spampleArm.rotateShoulderTo(80);
            }
            if (gamepad1.x){
                spampleArm.rotateShoulderTo(100);
            }
            
            
            telemetry.addData("arm angle", spampleArm.getArmAngle());
            
            
            
            spampleArm.updateArm();
            
            telemetry.update();
        }
    }
}
