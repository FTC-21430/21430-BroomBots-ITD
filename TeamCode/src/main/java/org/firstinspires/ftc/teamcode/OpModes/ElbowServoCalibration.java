package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Resources.InverseKinematics;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SampleCamera;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@TeleOp
public class ElbowServoCalibration extends LinearOpMode {
        int elbowPos = 0;

        Servo elbowServo = null;

        @Override
        public void runOpMode() throws InterruptedException {

            elbowServo = hardwareMap.get(Servo.class, "elbowServo");

            waitForStart();
            while (opModeIsActive()){

                if (gamepad1.dpad_up){
                    elbowServo.setPosition(1);
                }
                if (gamepad1.dpad_down){
                    elbowServo.setPosition(0);
                }

            telemetry.update();


            }
        }

}
