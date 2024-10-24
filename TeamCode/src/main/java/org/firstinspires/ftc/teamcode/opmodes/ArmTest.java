package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.systems.SpampleArm;

@TeleOp
public class ArmTest extends LinearOpMode {
    SpampleArm spampleArm;
    @Override
    public void runOpMode() throws InterruptedException {
        spampleArm = new SpampleArm(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
//            telemetry.update();
            if (gamepad2.a) {
                spampleArm.rotateWristTo(90);
            }
            if (gamepad2.b) {
                spampleArm.rotateWristTo(0);
            }
            if (gamepad2.x) {
                spampleArm.rotateWristTo(90);
            }
            if (gamepad2.y) {
                spampleArm.rotateWristTo(0);
            }

//            if (gamepad2.right_bumper) {
//                spampleArm.extendTo(5);
//            }
//            if (gamepad2.left_bumper) {
//                spampleArm.extendTo(0);
//            }
//            if (gamepad2.dpad_up) {
//                spampleArm.setClawPosition(Claw.ClawPosition.open);
//            }
//            if (gamepad2.dpad_down) {
//                spampleArm.setClawPosition(Claw.ClawPosition.closed);
//            }
        }
    }
}
