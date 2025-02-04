package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@Disabled
@TeleOp
public class shoulderCalibration extends BaseTeleOp {

        @Override
        public void runOpMode() throws InterruptedException {
            initialize(true, false);
            waitForStart();
            while (opModeIsActive()) {
                robot.spampleArm.currentArmState = SpampleArm.armState.test;

                telemetry.addData("arm angle: ", robot.spampleArm.getArmAngle());

                telemetry.update();


            }
        }
    }

