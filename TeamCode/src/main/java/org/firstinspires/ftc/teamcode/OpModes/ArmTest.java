package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@TeleOp
public class ArmTest extends LinearOpMode {
    SpampleArm spampleArm;
    @Override
    public void runOpMode() throws InterruptedException {
        spampleArm = new SpampleArm(hardwareMap, new ElapsedTime());
        waitForStart();
        while (opModeIsActive()) {
            
            if (gamepad1.a){
                spampleArm.setClawPosition(Claw.ClawPosition.closed);
            }
            if (gamepad1.b){
                spampleArm.setClawPosition(Claw.ClawPosition.grabInside);
            }
            if (gamepad1.x){
                spampleArm.setClawPosition(Claw.ClawPosition.open);
            }
            if (gamepad1.y){
                spampleArm.setClawPosition(Claw.ClawPosition.grabOutside);
            }
            
           
            telemetry.addData("arm angle", spampleArm.getArmAngle());
            telemetry.update();
        }
    }
}
