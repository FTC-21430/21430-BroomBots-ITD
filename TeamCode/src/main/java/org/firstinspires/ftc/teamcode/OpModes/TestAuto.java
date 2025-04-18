package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@Disabled
@TeleOp
@Config
public class TestAuto extends BaseAuto{
//
//public static double Pconstant= 0.15;
// public static double Iconstant= 0.1;
// public static double DConstant= 0.02;
// public static double speedMultplier=1;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, false);

        robot.driveTrain.setFieldCentricDriving(false);

        waitForStart();
        while (opModeIsActive()) {
            robot.setRobotSpeed(Robot.Speed.SLOW);
            if (gamepad1.a){
                robot.autoMoveTo(0,0,0,2,3);
            }
            if (gamepad1.b){
                robot.autoMoveTo(23.75,0,0,2,3);
            }
            if (gamepad1.x){
                robot.autoMoveTo(0, 0, 0, 2, 3);
            }
            if (gamepad1.y){
                robot.autoMoveTo(0,23.75,0,2,3);
            }
            if (gamepad1.dpad_down){
                robot.spampleArm.currentArmState= SpampleArm.armState.idle;
            }
            if(gamepad1.dpad_up){
                robot.spampleArm.currentArmState= SpampleArm.armState.highBasket;
            }
            robot.updateRobot(true, false, true);

            telemetry.addData("X",robot.odometry.getRobotX());
            telemetry.addData("Y", robot.odometry.getRobotY());
            telemetry.addData("Angle", robot.odometry.getRobotAngle());
            telemetry.update();

        }
    }
}
