package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@TeleOp
public class shoulderCalibration extends BaseTeleOp {

        @Override
        public void runOpMode() throws InterruptedException {
            initialize(true);
            waitForStart();
            while (opModeIsActive()) {
                robot.spampleArm.currentArmState = SpampleArm.armState.test;

                telemetry.addData("arm angle: ", robot.spampleArm.getArmAngle());

                telemetry.update();


            }
        }
    }

