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
            if (gamepad1.a){
                spampleArm.setClawPosition(Claw.ClawPosition.closed);
            }
            if (gamepad1.b){
                spampleArm.setClawPosition(Claw.ClawPosition.grabInside);
            }
            if (gamepad1.y){
                spampleArm.setClawPosition(Claw.ClawPosition.grabOutside);
            }
            if (gamepad1.x){
                spampleArm.setClawPosition(Claw.ClawPosition.open);
            }
//            if (gamepad1.a){
//                spampleArm.rotateTwistTo(90);
//            }
//            if (gamepad1.b){
//                spampleArm.rotateTwistTo(-90);
//            }
            
//            if (gamepad1.a){
//                spampleArm.extendTo(0);
//            }
//            if (gamepad1.b){
//                spampleArm.extendTo(10);
//            }
            
            //YAYYYAYYAYYYAYYYYAYYY
            
            if (gamepad1.a){
                spampleArm.rotateShoulderTo(45);
            }
            if (gamepad1.b){
                spampleArm.rotateShoulderTo(0);
            }
            
            spampleArm.updateSlide();
            
            telemetry.update();
            
            
        }
    }
}
