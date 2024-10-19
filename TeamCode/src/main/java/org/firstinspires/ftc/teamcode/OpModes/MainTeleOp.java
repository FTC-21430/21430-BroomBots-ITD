package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.teamcode.Resources.Utlities;
//This is the teleop we run during competitions.
@TeleOp
public class MainTeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        // We multiply this by the speed to activate slowmode.
        final double slowSpeedMultiplier = 0.40;
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            // Sets slow mode if right bumper is pressed.
            if (gamepad1.right_bumper) {
                robot.driveTrain.setSpeedMultiplier(slowSpeedMultiplier);
            } else{
                robot.driveTrain.setSpeedMultiplier(1);

            robot.odometry.updateOdometry();
            /** Target angle gives the angle from -180 to 180 that the robot wants to be at
             *
             * Wrap makes the angle reset at -180 or 180 when we make a full rotation
             * robot.anglePID.getTarget() is our current angle
             * The driver input is equal to the right joystick
             * RETEST!!! maxTurnDegPerSecond is the fastest we can turn
             * robot.getDeltaTime gets the in time for each run through of our code
             * Speed multiplier changes how fast the robot is during slow mode
             *
             * We multiply the variables by each other and add it to our current angle to determine our new target angle
             */
            double targetAngle = Utlities.wrap(robot.anglePID.getTarget() + (-gamepad1.right_stick_x * robot.maxTurnDegPerSecond * robot.getDeltaTime() * robot.driveTrain.getSpeedMultiplier()));

            // These call functions and pass the relevant parameters
            robot.anglePID.setTarget(targetAngle);
            robot.anglePID.update(robot.odometry.getRobotAngle());
            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.anglePID.getPower(), robot.odometry.getRobotAngle());

            // Telemetry for testing purposes
            telemetry.addData("Robot Angle",robot.odometry.getRobotAngle());
            telemetry.addData("Target Angle",targetAngle);
            telemetry.update();

            }
        }
    }
}
