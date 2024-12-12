package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.InverseKinematics;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@TeleOp
public class ArmTest extends BaseTeleOp {
    

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            robot.spampleArm.currentArmState = SpampleArm.armState.test;
            if (gamepad1.a){
                robot.spampleArm.rotateElbowTo(0);
            }
            if (gamepad1.b){
                robot.spampleArm.rotateElbowTo(90);
            }
            if (gamepad1.x){
                robot.spampleArm.rotateElbowTo(30);
            }
            if (gamepad1.y){
                robot.spampleArm.rotateElbowTo(60);
            }
            if (gamepad1.dpad_up){
                robot.spampleArm.rotateElbowTo(-90);
            }
            if (gamepad1.dpad_right) {
                robot.spampleArm.rotateElbowTo(-120);
            }
            if (gamepad1.dpad_down){
                robot.spampleArm.rotateElbowTo(95);
            }
            robot.updateRobot(false, false);
            telemetry.update();


        }
    }
}
