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

            robot.driveTrain.setSpeedMultiplier(0.4);

            robot.updateLoopTime();

            robot.odometry.updateOdometry();

//            robot.spampleArm.currentArmState = robot.spampleArm.armState.idle;
            if (gamepad1.a){
                robot.autoMoveTo(0,23.75, 0, 1);
            }
            if (gamepad1.b){
                robot.autoMoveTo(0,0,0,1);
            }



            robot.driveTrain.setDrivePower(0,0,0,robot.odometry.getRobotAngle());
            robot.updateRobot(true, true);


            robot.spampleArm.updateArm();
            telemetry.addData("X",robot.odometry.getRobotX());
            telemetry.addData("Y", robot.odometry.getRobotY());
            telemetry.addData("Angle", robot.odometry.getRobotAngle());
            telemetry.update();

        }
    }
}
