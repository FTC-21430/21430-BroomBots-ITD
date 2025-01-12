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

            if (gamepad1.dpad_up){
                robot.spampleArm.extendTo(10);
            } else if (gamepad1.dpad_down) {
                robot.spampleArm.extendTo(0);
            } else if (gamepad1.dpad_right) {
                robot.spampleArm.extendTo(18.5);
            }


            if (gamepad1.a){
                robot.spampleArm.rotateShoulderTo(90);
            }
            if (gamepad1.b){
                robot.spampleArm.rotateShoulderTo(110);
            }
            if (gamepad1.x){
                robot.spampleArm.rotateShoulderTo(70);
            }

            if (gamepad1.right_trigger > 0.6){
                robot.updateRobot(false, false);
            }else{
                robot.spampleArm.shoulderMotor.setPower(0);
            }
            telemetry.addData("arm angle", robot.spampleArm.getArmAngle());
            telemetry.addData("pid power", robot.spampleArm.shoulderPID.getPower());

            telemetry.addData("pid target", robot.spampleArm.shoulderPID.getTarget());

            telemetry.update();


        }
    }
}
