package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.InverseKinematics;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@TeleOp
public class ArmTest extends LinearOpMode {
    SpampleArm spampleArm;
    InverseKinematics kinematics;
    @Override
    public void runOpMode() throws InterruptedException {
        
        spampleArm = new SpampleArm(hardwareMap, new ElapsedTime());
        waitForStart();
        while (opModeIsActive()) {

//            if (gamepad1.a){
//                spampleArm.setClawPosition(Claw.ClawPosition.closed);
//            }
//            if (gamepad1.b){
//                spampleArm.setClawPosition(Claw.ClawPosition.grabInside);
//            }
//            if (gamepad1.y){
//                spampleArm.setClawPosition(Claw.ClawPosition.grabOutside);
//            }
//            if (gamepad1.x){
//                spampleArm.setClawPosition(Claw.ClawPosition.open);
//            }
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

            if (gamepad1.y){
                spampleArm.rotateShoulderTo(30);
            }
            if (gamepad1.a){
                spampleArm.rotateShoulderTo(90);
            }
            if (gamepad1.b){
                spampleArm.rotateShoulderTo(60);
            }
            if (gamepad1.x){
                spampleArm.rotateShoulderTo(130);
            }

//            if (gamepad1.dpad_down){
//                spampleArm.rotateTwistTo(0);
//            }
//            if (gamepad1.dpad_up){
//                spampleArm.rotateTwistTo(90);
//            }
//            if (gamepad1.dpad_left){
//                spampleArm.rotateTwistTo(180);
//
//            }
           

            telemetry.addData("arm angle", spampleArm.getArmAngle());
            
            spampleArm.updateSlide();
            
            telemetry.update();
        }
    }
}
