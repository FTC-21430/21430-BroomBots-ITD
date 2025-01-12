package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Resources.InverseKinematics;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;


//This is the teleop we run during competitions.
@TeleOp
public class MainTeleOp extends BaseTeleOp {


    InverseKinematics kinematics;
    boolean gp1xJoy = false;
    boolean gp2tri = false;
    public double LastLoopTime;
    double targetAngle = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // We multiply this by the speed to activate slowmode.
        final double slowSpeedMultiplier = 0.35;

        initialize();

        telemetry.setMsTransmissionInterval(10);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        kinematics = new InverseKinematics();
        waitForStart();
        while (opModeIsActive()) {

            robot.updateLoopTime();

            // get and update functions
            robot.odometry.updateOdometry();

            //TODO: fix the april tag system
//            robot.aprilTags.findAprilTags(robot.odometry.getRobotX(),robot.odometry.getRobotY());
//            if (robot.aprilTags.hasDetection()){
//                // to ensure that if we get a wildly wrong pos from a tag, we don't use it. it is ok if within 2 inches
//                if (robot.aprilTags.calculateDistance(robot.odometry.getRobotX(),robot.odometry.getRobotY(), robot.aprilTags.getRobotX(),robot.aprilTags.getRobotY()) < 2){
//                    robot.odometry.overridePosition(robot.aprilTags.getRobotX(),robot.aprilTags.getRobotY(), robot.aprilTags.getRobotYaw());
//                }
//            }


            // resets Field Centric Driving

            if (gamepad1.share) {
                robot.IMUReset();
            }

            //  claw Positions

            if (gamepad1.cross) {
                robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
            }
            if (gamepad1.circle) {
                robot.spampleArm.setClawPosition(Claw.ClawPosition.grabInside);
            }
            if (gamepad1.triangle) {
                robot.spampleArm.setClawPosition(Claw.ClawPosition.grabOutside);
            }
            if (gamepad1.square) {
                robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
            }

            // angles for testing only

//                if (gamepad1.cross){
//                    robot.spampleArm.rotateShoulderTo(90);
//                }
//                if (gamepad1.square){
//                    robot.spampleArm.rotateShoulderTo(30);
//                }
//                if (gamepad1.triangle){
//                    robot.spampleArm.rotateShoulderTo(70);
//                }
//                if (gamepad1.circle){
//                    robot.spampleArm.rotateShoulderTo(150);
//                }
            // lengths for testing only
//                if (gamepad1.dpad_up){
//                    robot.spampleArm.extendTo(0);
//                }
//                if (gamepad1.dpad_left){
//                    robot.spampleArm.extendTo(19);
//                }
//                if (gamepad1.dpad_right){
//                    robot.spampleArm.extendTo(6);
//                }
//                if (gamepad1.dpad_down){
//                    robot.spampleArm.extendTo(12);
//                }


            // arm positions

            if (gamepad2.dpad_up) {
                robot.spampleArm.currentArmState = SpampleArm.armState.highBasket;
            }
            if (gamepad2.triangle && !gp2tri) {
                if (robot.spampleArm.currentArmState == SpampleArm.armState.highChamber) {
//                    robot.spampleArm.currentArmState = SpampleArm.armState.scoreHighChamber;
                } else {
                    robot.spampleArm.currentArmState = SpampleArm.armState.highChamber;
                }
            }
            gp2tri = gamepad2.triangle;

            if (gamepad2.cross) {
                robot.spampleArm.currentArmState = SpampleArm.armState.lowChamber;
            }
            if (gamepad2.square) {
                robot.spampleArm.currentArmState = SpampleArm.armState.lowBasket;
            }
            if (gamepad2.dpad_left) {
                robot.spampleArm.currentArmState = SpampleArm.armState.grabSpecimen;
            }
            if (gamepad2.dpad_down) {
                if (robot.spampleArm.currentArmState == SpampleArm.armState.grabSpecimen) {
                    robot.spampleArm.currentArmState = SpampleArm.armState.specimenIdle;
                } else {
                    robot.spampleArm.currentArmState = SpampleArm.armState.idle;
                }

                robot.spampleArm.currentArmState = SpampleArm.armState.idle;
            }
            if (gamepad2.dpad_right) {
//                robot.spampleArm.currentArmState = SpampleArm.armState.climberReady;
            }
            if (gamepad2.right_bumper) {

                robot.spampleArm.saveShoulderTime();
                robot.spampleArm.currentArmState = SpampleArm.armState.grabSample;
            }
            if (gamepad2.right_trigger > 0.3) {

                robot.spampleArm.currentArmState = SpampleArm.armState.grabSample2;
            }
//                if (gamepad1.dpad_right){
//                    currentArmState = armState.spearHead;
//                }


            if (gamepad1.left_bumper) {
                robot.spampleArm.currentArmState = SpampleArm.armState.idle;
            }

            if (robot.spampleArm.getTwist() <= 90 && robot.spampleArm.getTwist() >= -90) {
                robot.spampleArm.rotateTwistTo(robot.spampleArm.getTwist() + gamepad2.right_stick_x * robot.getDeltaTime() * 180);
            } else if (robot.spampleArm.getTwist() > 90) {
                robot.spampleArm.rotateTwistTo(90);
            } else if (robot.spampleArm.getTwist() < -90) {
                robot.spampleArm.rotateTwistTo(-90);
            }





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
            targetAngle = (robot.anglePID.getTarget() + (-gamepad1.right_stick_x * robot.maxTurnDegPerSecond * robot.getDeltaTime() * robot.driveTrain.getSpeedMultiplier()));

//                if (Math.abs(gamepad1.right_stick_x) < 0.1 == false && gp1xJoy == true) {
//                    targetAngle = robot.odometry.getRobotAngle();
//                }

            // These call functions and pass the relevant parameters
            robot.anglePID.setTarget(targetAngle);


            robot.anglePID.update(robot.odometry.getRobotAngle());

            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                if (gamepad1.dpad_up) {
                    robot.driveTrain.setDrivePower(1, 0, robot.anglePID.getPower(), robot.odometry.getRobotAngle());
                }
                if (gamepad1.dpad_right) {
                    robot.driveTrain.setDrivePower(0, 1, robot.anglePID.getPower(), robot.odometry.getRobotAngle());
                }
                if (gamepad1.dpad_down) {
                    robot.driveTrain.setDrivePower(-1, 0, robot.anglePID.getPower(), robot.odometry.getRobotAngle());
                }
                if (gamepad1.dpad_left) {
                    robot.driveTrain.setDrivePower(0, -1, robot.anglePID.getPower(), robot.odometry.getRobotAngle());
                }
            } else {
                robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.anglePID.getPower(), robot.odometry.getRobotAngle());
            }


//            if (gamepad1.right_trigger >= 0.5){
//                robot.climber.extendTo(12.50);
//            }else{
//                robot.climber.startingPosition();
//            }

            robot.spampleArm.updateArm();

            robot.updateRobot(false, true);
            if (gamepad1.right_bumper) {
                robot.driveTrain.setSpeedMultiplier(slowSpeedMultiplier);
            }

            // the old input for the left stick x axis for gamepad 1,
            // updated at the end of the loop so the turning logic works :)
            gp1xJoy = gamepad1.right_stick_x > 0.1;


            // Telemetry for testing/debug purposes

            telemetry.addData("arm extension", robot.spampleArm.getArmExtension());

            telemetry.addData("current Speed", robot.driveTrain.getSpeedMultiplier());
            telemetry.addData("currentState", robot.spampleArm.currentArmState);
            telemetry.addData("Robot Angle", robot.odometry.getRobotAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("arm angle", robot.spampleArm.getArmAngle());
            telemetry.addData("Robot X", robot.odometry.getRobotX());
            telemetry.addData("Robot Y", robot.odometry.getRobotY());
            telemetry.addData("Potemeter", robot.spampleArm.getArmAngle());
            double currentlooptime = getRuntime();
            telemetry.addData("Runtime", currentlooptime - LastLoopTime);
            LastLoopTime = currentlooptime;
            telemetry.addData("MoterPower", robot.spampleArm.shoulderMotor.getPower());


            telemetry.addData("UpperGraph", 100);
            telemetry.addData("LowerGraph", 0);
            telemetry.addData("TargetAngle", robot.spampleArm.shoulderPID.target);
            telemetry.addData("MeasuedAngle", robot.spampleArm.getArmAngle());
            telemetry.addData("Power", robot.spampleArm.shoulderMotor.getPower());
            telemetry.addData("PowerProportional", robot.spampleArm.shoulderPID.powerProportional);
            telemetry.addData("PowerIntegral", robot.spampleArm.shoulderPID.powerIntegral);
            telemetry.addData("PowerDerivative", robot.spampleArm.shoulderPID.powerDerivative);
            telemetry.update();


        }
    }
}
