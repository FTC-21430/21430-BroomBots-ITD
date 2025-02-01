package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Resources.InverseKinematics;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SampleCamera;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;


//This is the teleop we run during competitions.
@TeleOp
public class MainTeleOp extends BaseTeleOp {
    boolean climberActive = false;
    boolean climberSwitchPrev = false;
    private InverseKinematics kinematics;
    boolean gp1xJoy = false;
    boolean gp2tri = false;
    public double LastLoopTime;
    double targetAngle = 0;

    private SampleCamera sampleCamera;
    private double target_r_rot;
    private double extension;
    private double elbow;
    private double twist;
    private double shoulder_rot;


    private double foundX;
    private double foundY;
    private double foundYaw;

    private boolean lookingForSample;
    private double startedLookingTime = 0.0;
    private double searchTimeout = 1.0;
    private boolean grabbingSample = false;
    private double grabZ = 0.0;

    private double alignmentTime = 0.6;

    private double loweringTime = 0.5;

    private double grabbingTime = 0.3;

    private double startingAngle = 0;

    private boolean aligning = false;
    private boolean lowering = false;
    private boolean grabbing = false;
    private double autoPickupTimer = 0.0;

    private boolean manualMode = false;
    private boolean gp2shareold = false;


    @Override
    public void runOpMode() throws InterruptedException {
        // We multiply this by the speed to activate slowmode.
        final double slowSpeedMultiplier = 0.35;
        final double updateTagsSpeed = 0.15;

        initialize(false);

        telemetry.setMsTransmissionInterval(10);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);

        sampleCamera = new SampleCamera(hardwareMap, telemetry);

        kinematics = new InverseKinematics();
        waitForStart();
        while (opModeIsActive()) {

            robot.updateLoopTime();

            // get and update functions
            robot.odometry.updateOdometry();



//            robot.aprilTags.findAprilTags(robot.odometry.getRobotX(),robot.odometry.getRobotY());
//            // checks if there is a tag to check and that we are moving slow enough to limit using motion blur of the tag
//            if (robot.aprilTags.hasDetection() && robot.driveTrain.getAvgDrivePower() <= updateTagsSpeed){
//                // to ensure that if we get a wildly wrong pos from a tag, we don't use it. it is ok if within 2 inches
//                if (robot.aprilTags.calculateDistance(robot.odometry.getRobotX(),robot.odometry.getRobotY(), robot.aprilTags.getRobotX(),robot.aprilTags.getRobotY()) < 2){
////                    robot.odometry.overridePosition(robot.aprilTags.getRobotX(),robot.aprilTags.getRobotY(), robot.aprilTags.getRobotYaw());
//                }
//            }


            // resets Field Centric Driving

            if (gamepad1.share) {
                robot.IMUReset();
            }

            if (gamepad2.share && !gp2shareold){
                if (manualMode){
                    manualMode = false;
                }else{
                    manualMode = true;
                }
            }
            gp2shareold = gamepad2.share;

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

            // arm positions

            if (!lookingForSample && !grabbingSample){
                if (gamepad2.dpad_up) {
                    robot.spampleArm.currentArmState = SpampleArm.armState.highBasket;
                }
                if (gamepad2.triangle && !gp2tri) {
                    if (robot.spampleArm.currentArmState == SpampleArm.armState.highChamber) {
                    } else {
                        robot.spampleArm.rotateElbowTo(88.5);
                    }
                    robot.spampleArm.currentArmState = SpampleArm.armState.highChamber;
                }
                if (robot.spampleArm.currentArmState == SpampleArm.armState.highChamber){
                    if (robot.spampleArm.getElbowRotation() <= 94 && robot.spampleArm.getElbowRotation() >= 84) {
                        robot.spampleArm.rotateElbowTo(robot.spampleArm.getElbowRotation() + gamepad2.left_stick_y * robot.getDeltaTime() * -30);
                    } else if (robot.spampleArm.getElbowRotation() > 94) {
                        robot.spampleArm.rotateElbowTo(94);
                    } else if (robot.spampleArm.getElbowRotation() < 84) {
                        robot.spampleArm.rotateElbowTo(84);
                    }
                }

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
                    robot.spampleArm.currentArmState = SpampleArm.armState.idle;

                }

                if (manualMode) {
                    if (gamepad2.right_bumper) {

                        robot.spampleArm.saveShoulderTime();
                        robot.spampleArm.currentArmState = SpampleArm.armState.grabSample;
                    }
                    if (gamepad2.right_trigger > 0.3) {

                        robot.spampleArm.currentArmState = SpampleArm.armState.grabSample2;
                    }
                }
//
                if (gamepad2.dpad_right){
                    robot.spampleArm.currentArmState = SpampleArm.armState.climberReady;
                }

                if (robot.spampleArm.getTwist() <= 90 && robot.spampleArm.getTwist() >= -90) {
                    robot.spampleArm.rotateTwistTo(robot.spampleArm.getTwist() + gamepad2.right_stick_x * robot.getDeltaTime() * 180);
                } else if (robot.spampleArm.getTwist() > 90) {
                    robot.spampleArm.rotateTwistTo(90);
                } else if (robot.spampleArm.getTwist() < -90) {
                    robot.spampleArm.rotateTwistTo(-90);
                }
            }


            if (!manualMode) {
                if (gamepad2.right_bumper) {
                    robot.spampleArm.currentArmState = SpampleArm.armState.pictureTake;

                    if (grabbingSample || lookingForSample){
                        robot.anglePID.setTarget(startingAngle);
                    }

                    grabbingSample = false;
                    lookingForSample = false;
                    sampleCamera.stopDetection();
                    robot.setTurnPIntake(false);

                }

                if (robot.spampleArm.currentArmState == SpampleArm.armState.pictureTake) {

                    if (gamepad1.dpad_down) {
                        lookingForSample = true;
                        sampleCamera.findYellowSample();
                        startedLookingTime = runtime.time();
                    }
                    if (gamepad1.dpad_left) {
                        lookingForSample = true;
                        sampleCamera.findRedSample();
                        startedLookingTime = runtime.time();
                    }
                    if (gamepad1.dpad_right) {
                        lookingForSample = true;
                        sampleCamera.findBlueSample();
                        startedLookingTime = runtime.time();
                    }
                }

                if (lookingForSample) {
                    if (runtime.time() > startedLookingTime + searchTimeout) {
                        lookingForSample = false;
                        gamepad1.rumble(0.6, 0.6, 500);
                    } else if (sampleCamera.didWeFindOne()) {
                        lookingForSample = false;

                        sampleCamera.findCameraPosRelativePosition(robot.spampleArm.getArmAngle(), robot.spampleArm.getArmExtension(), 0);

                        sampleCamera.stopDetection();

                        foundX = sampleCamera.getSampleX();
                        foundY = sampleCamera.getSampleY();
                        foundYaw = sampleCamera.getSampleYaw();

                        if (kinematics.calculateKinematics(0, 0, foundX, foundY, grabZ, foundYaw, robot.odometry.getRobotAngle())) {

                            gamepad1.rumble(0.0, 0.5, 500);

                            target_r_rot = kinematics.getRobotAngle() + robot.odometry.getRobotAngle();
                            shoulder_rot = kinematics.getArmRotation();
                            elbow = kinematics.getElbowRotation();
                            extension = kinematics.getArmExtension();
                            twist = kinematics.getTwist();

                            grabbingSample = true;
                            startingAngle = robot.odometry.getRobotAngle();

                        } else {
                            gamepad1.rumble(0.5, 0.0, 800);
                        }
                    }
                }
            }else{
                lookingForSample = false;
                grabbingSample = false;
            }

            gp2tri = gamepad2.triangle;
            //Climber operation
            if (gamepad1.left_bumper && !climberSwitchPrev){
                if (!robot.climber.getIfInitilized()){
                    robot.climber.initClimber();
                }
                if (climberActive){
                    climberActive = false;
                }else{
                    climberActive = true;
                }
            }

            climberSwitchPrev = gamepad1.left_bumper;

            if (climberActive){
                robot.climber.releaseLatches();

                if (gamepad1.left_trigger > 0.6){
                    robot.climber.extendTo(12.5);
                }else{
                    robot.climber.extendTo(0);
                }
            }else{
                robot.climber.lockLatches();
                robot.climber.extendTo(0);
            }

            telemetry.addData("climberActive", climberActive);
            telemetry.addData("climber init?", robot.climber.getIfInitilized());




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

            if (!grabbingSample && !lookingForSample) {
                targetAngle = (robot.anglePID.getTarget() + (-gamepad1.right_stick_x * robot.maxTurnDegPerSecond * robot.getDeltaTime() * robot.driveTrain.getSpeedMultiplier()));

//                if (Math.abs(gamepad1.right_stick_x) < 0.1 == false && gp1xJoy == true) {
//                    targetAngle = robot.odometry.getRobotAngle();
//                }

                // These call functions and pass the relevant parameters
                robot.anglePID.setTarget(targetAngle);


                robot.anglePID.update(robot.odometry.getRobotAngle());

                robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.anglePID.getPower(), robot.odometry.getRobotAngle());


//            if (gamepad1.right_trigger >= 0.5){
//                robot.climber.extendTo(12.50);
//            }else{
//                robot.climber.startingPosition();
//            }

//                robot.spampleArm.updateArm();

                 if (gamepad1.right_bumper) {
                    robot.driveTrain.setSpeedMultiplier(slowSpeedMultiplier);
                }

            }
//            else if (grabbingSample) {
                else if(false){
                if (!grabbing && !lowering && !aligning) {
                    robot.setTurnPIntake(true);
                    robot.spampleArm.currentArmState = SpampleArm.armState.test;
                    robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
                    autoPickupTimer = runtime.time();

                    aligning = true;
                    grabbingSample = true;
                    startingAngle = robot.odometry.getRobotAngle();
                    robot.anglePID.setTarget(target_r_rot);
                    robot.spampleArm.rotateShoulderTo(shoulder_rot + 15);
                    robot.spampleArm.rotateElbowTo(elbow);
                    robot.spampleArm.rotateTwistTo(twist);
                    robot.spampleArm.extendTo(extension);
                }
                else if (aligning) {
                    if (runtime.time() > autoPickupTimer + alignmentTime) {
                        aligning = false;
                        lowering = true;
                        autoPickupTimer = runtime.time();
                        grabbingSample = true;
                        robot.anglePID.setTarget(target_r_rot);
                        robot.spampleArm.rotateShoulderTo(shoulder_rot);
                        robot.spampleArm.rotateElbowTo(elbow);
                        robot.spampleArm.rotateTwistTo(twist);
                        robot.spampleArm.extendTo(extension);

                    }
                } else if (lowering) {
                    if (runtime.time() > autoPickupTimer + loweringTime) {
                        robot.spampleArm.setClawPosition(Claw.ClawPosition.grabInside);
                        lowering = false;
                        grabbing = true;
                        autoPickupTimer = runtime.time();
                    }
                } else if (grabbing) {
                    if (runtime.time() > autoPickupTimer + grabbingTime) {
                        robot.spampleArm.currentArmState = SpampleArm.armState.pictureTake;
                        grabbing = false;
                        grabbingSample = false;
                        robot.setTurnPIntake(false);
                    }
                }
                robot.driveTrain.setSpeedMultiplier(0.8);
                robot.anglePID.update(robot.odometry.getRobotAngle());

                robot.driveTrain.setDrivePower(0, 0, robot.anglePID.getPower(), robot.odometry.getRobotAngle());

//                robot.spampleArm.updateArm();
            }else {
                robot.anglePID.update(robot.odometry.getRobotAngle());
                robot.driveTrain.setDrivePower(0, 0, robot.anglePID.getPower(), robot.odometry.getRobotAngle());
            }

            robot.updateRobot(false, false);

            // the old input for the left stick x axis for gamepad 1,
            // updated at the end of the loop so the turning logic works :)
            gp1xJoy = gamepad1.right_stick_x > 0.1;



            // Telemetry for testing/debug purposes


            // did that to turn off these telemetry lines for the  auto pickup sample testing - Tobin - 1/30/25
            if (false) {
                telemetry.addData("elbow angle", robot.spampleArm.getElbowRotation());

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


            }
            else{
                telemetry.addLine("SampleX: " + foundX);
                telemetry.addLine("SampleY: " + foundY);
                telemetry.addLine("SampleYaw: " + foundYaw);
                telemetry.addLine("------------");
                telemetry.addLine("camera X: " + sampleCamera.cameraXRobot);
                telemetry.addLine("camera Y: " + sampleCamera.cameraYRobot);
                telemetry.addLine("camera Z: " + sampleCamera.cameraZRobot);

                telemetry.addLine("------");
                telemetry.addLine("target_r_rot: " + kinematics.getRobotAngle());
                telemetry.addLine("shoulder rot: " + kinematics.getArmRotation());
                telemetry.addLine("elbow: " + kinematics.getElbowRotation());
                telemetry.addLine("twisty twist: " + kinematics.getTwist());
                telemetry.addLine("extension: " + kinematics.getArmExtension());

            }
            telemetry.update();
            grabbingSample = false;
        }
    }
}
