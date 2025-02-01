package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Resources.InverseKinematics;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SampleCamera;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

/**
 * Started this new class to be built upon the automated sample detection.
 * Use this in competition
 */

@TeleOp
public class CompetitionTeleop extends BaseTeleOp {

    // The camera class for the camera on the arm
    private SampleCamera sampleCamera;

    // the Inverse Kinematics instance for calculating arm rotations for sample positions
    private InverseKinematics kinematics;

    // The output of the Inverse Kinematics to be used elsewhere in the file
    private double target_r_rot;
    private double extension;
    private double elbow;
    private double twist;
    private double shoulder_rot;

    // The found sample positions
    private double foundX;
    private double foundY;
    private double foundYaw;

    // the logic for the auto grabs and detection
    private boolean lookingForSample;
    private double startedLookingTime = 0.0;
    private boolean grabbingSample = false;
    private boolean aligning = false;
    private boolean lowering = false;
    private boolean grabbing = false;

    // the height we pick up samples
    private final double grabZ = 0.0;

    // Timings for auto pickups
    private double searchTimeout = 0.7;
    private double alignmentTime = 1.0;
    private double loweringTime = 0.5;
    private double grabbingTime = 0.3;
    private double startingAngle = 0;
    private double autoPickupTimer = 0.0;

    // Teleop Logic
    final double slowSpeedMultiplier = 0.35;
    final double midSpeedMultiplier = 0.65;

    // the angle the robot should be facing
    double targetAngle = 0;

    /** manual mode is for when auto pickups are
     not working and the drivers need manual control
     */
    private boolean manualMode = false;

    // for if the share button was pressed last iteration on controller 2
    private boolean gp2shareold = false;

    // stores if we are currently inputting a turn on controller 1
    boolean gp1xJoy = false;
    boolean gp2tri = false;

    // for getting our loop times
    public double LastLoopTime;

    // climber logic:

    // climberActive locks the abiltiy to extend the climbers until driver 1 activates them when ready.
    boolean climberActive = false;

    // for ensuring that climberActive only switches once a push of the button
    boolean climberSwitchPrev = false;

    // The main code that runs during init
    @Override
    public void runOpMode() throws InterruptedException {

        // initializes the robot without resetting the odometry
        initialize(true);

        // creates the new classes for the camera and kinematics
        sampleCamera = new SampleCamera(hardwareMap, telemetry);
        kinematics = new InverseKinematics();

        waitForStart();
        while(opModeIsActive()) {

            // get and update functions
            robot.updateLoopTime();
            robot.odometry.updateOdometry();

            // resets Field Centric Driving
            if (gamepad1.share) {
                robot.IMUReset();
            }

            // toggles manual mode
            if (gamepad2.share && !gp2shareold){
                if (manualMode){
                    manualMode = false;
                }else{
                    manualMode = true;
                }
            }
            manualMode = false;

//          claw logic
            if (gamepad1.cross) {
                robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
            }
            if (gamepad1.square) {
                robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
            }
            if (gamepad1.triangle) {
                robot.spampleArm.setClawPosition(Claw.ClawPosition.grabOutside);
            }
            if (gamepad1.circle) {
                robot.spampleArm.setClawPosition(Claw.ClawPosition.grabInside);
            }


            // Arm control

            if (manualMode){
                if (gamepad2.right_bumper) {

                    robot.spampleArm.saveShoulderTime();
                    robot.spampleArm.currentArmState = SpampleArm.armState.grabSample;
                }
                if (gamepad2.right_trigger > 0.4) {

                    robot.spampleArm.currentArmState = SpampleArm.armState.grabSample2;
                }
            }else if (gamepad2.right_bumper) {
                grabbingSample = false;
                lookingForSample = false;
                sampleCamera.stopDetection();
                robot.spampleArm.currentArmState = SpampleArm.armState.pictureTake;
                if(lookingForSample || grabbingSample){
                    robot.anglePID.setTarget(startingAngle);
                }
                extension = 2;
                target_r_rot = 0;
                shoulder_rot = 30;
                twist = 0;
                elbow = 60;

                robot.setTurnPIntake(false);

            }
            if (!grabbingSample || !lookingForSample) {

                if (gamepad2.dpad_down && !grabbingSample) {
                    robot.spampleArm.currentArmState = SpampleArm.armState.idle;
                    robot.setTurnPIntake(false);
                }
            }



            if (robot.spampleArm.currentArmState == SpampleArm.armState.pictureTake) {
                // starts the vision detection process in either red, blue or yellow.
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
            // if we should be looking for a sample, we check if we should stop.

            if (lookingForSample) {

                // if we time out the detection
                if (runtime.time() > startedLookingTime + searchTimeout) {
                    lookingForSample = false;
                    // rumbles the whole controller to tell driver 1 that we could not find a sample
                    gamepad1.rumble(0.5,0.0, 700);
                }
                // checks if we found a sample already so we don't wait unnecessarily
                else if (sampleCamera.didWeFindOne()) {
                    sampleCamera.acceptBuffer();
                    sampleCamera.stopDetection();
                    telemetry.addData("I found it!: ", sampleCamera.didWeFindOne());

                    lookingForSample = false;

                    sampleCamera.findCameraPosRelativePosition(robot.spampleArm.getArmAngle(), robot.spampleArm.getArmExtension(), 0);
//
//

                    foundX = sampleCamera.getSampleX();
                    foundY = sampleCamera.getSampleY();
                    foundYaw = sampleCamera.getSampleYaw();

                    telemetry.addData("No, I did!: ", sampleCamera.didWeFindOne());
                    telemetry.addLine("SampleX: " + foundX);
                    telemetry.addLine("SampleY: " + foundY);
                    telemetry.addLine("SampleYaw: " + foundYaw);
                    telemetry.addLine("------------");
                    telemetry.addLine("camera X: " + sampleCamera.cameraXRobot);
                    telemetry.addLine("camera Y: " + sampleCamera.cameraYRobot);
                    telemetry.addLine("camera Z: " + sampleCamera.cameraZRobot);



                    if (kinematics.calculateKinematics(0, 0, foundX, foundY, grabZ, foundYaw, robot.odometry.getRobotAngle())) {

                        gamepad1.rumble(0.0, 0.3, 700);

                        target_r_rot = kinematics.getRobotAngle() + robot.odometry.getRobotAngle();
                        shoulder_rot = kinematics.getArmRotation();
                        elbow = kinematics.getElbowRotation();
                        extension = kinematics.getArmExtension();
                        twist = kinematics.getTwist();

                        grabbingSample = true;
                        startingAngle = robot.odometry.getRobotAngle();

                        telemetry.addLine("------");
                        telemetry.addLine("target_r_rot: " + kinematics.getRobotAngle());
                        telemetry.addLine("shoulder rot: " + kinematics.getArmRotation());
                        telemetry.addLine("elbow: " + kinematics.getElbowRotation());
                        telemetry.addLine("twisty twist: " + kinematics.getTwist());
                        telemetry.addLine("extension: " + kinematics.getArmExtension());

                    } else {
                        gamepad1.rumble(0.8,0.0, 700);
                    }
                    telemetry.update();

                }
            }

            sampleCamera.clearFoundSample();

            if (!grabbingSample && !lookingForSample || manualMode) {
                targetAngle = (robot.anglePID.getTarget() + (-gamepad1.right_stick_x * robot.maxTurnDegPerSecond * robot.getDeltaTime() * robot.driveTrain.getSpeedMultiplier()));

                robot.driveTrain.setSpeedMultiplier(0.8);

                // These call functions and pass the relevant parameters
                robot.anglePID.setTarget(targetAngle);
                robot.anglePID.update(robot.odometry.getRobotAngle());

                if (gamepad1.right_bumper) {
                    robot.driveTrain.setSpeedMultiplier(slowSpeedMultiplier);
                }else if(gamepad1.right_trigger > 0.4){
                    robot.driveTrain.setSpeedMultiplier(midSpeedMultiplier);
                }

                robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.anglePID.getPower(), robot.odometry.getRobotAngle());

                robot.updateRobot(false, false);

            } else if (grabbingSample) {
                if (!grabbing && !lowering && !aligning) {
                    robot.setTurnPIntake(true);
                    robot.spampleArm.currentArmState = SpampleArm.armState.test;
                    robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
                    autoPickupTimer = runtime.seconds();

                    aligning = true;

                    startingAngle = robot.odometry.getRobotAngle();
                    robot.anglePID.setTarget(target_r_rot);
                    robot.spampleArm.rotateShoulderTo(shoulder_rot+15);
                    robot.spampleArm.rotateElbowTo(elbow);
                    robot.spampleArm.rotateTwistTo(twist);
                    robot.spampleArm.extendTo(extension);
                } else if (aligning) {
                    if (runtime.seconds() > autoPickupTimer + alignmentTime) {
                        aligning = false;
                        lowering = true;
                        autoPickupTimer = runtime.seconds();
                        grabbingSample = true;
                        robot.anglePID.setTarget(target_r_rot);
                        robot.spampleArm.rotateShoulderTo(shoulder_rot);
                        robot.spampleArm.rotateElbowTo(elbow);
                        robot.spampleArm.rotateTwistTo(twist);
                        robot.spampleArm.extendTo(extension);

                    }
                } else if (lowering) {
                    if (runtime.seconds() > autoPickupTimer + loweringTime) {
                        robot.spampleArm.setClawPosition(Claw.ClawPosition.grabInside);
                        lowering = false;
                        grabbing = true;
                        autoPickupTimer = runtime.seconds();
                    }
                } else if (grabbing) {
                    if (runtime.seconds() > autoPickupTimer + grabbingTime) {
                        robot.spampleArm.currentArmState = SpampleArm.armState.pictureTake;
                        grabbing = false;
                        grabbingSample = false;
                        robot.setTurnPIntake(false);
                    }
                }
                robot.driveTrain.setSpeedMultiplier(0.8);
                robot.anglePID.update(robot.odometry.getRobotAngle());

                robot.driveTrain.setDrivePower(0, 0, robot.anglePID.getPower(), robot.odometry.getRobotAngle());

                robot.updateRobot(false, false);

            }else{
                robot.anglePID.update(robot.odometry.getRobotAngle());

                robot.driveTrain.setDrivePower(0, 0, robot.anglePID.getPower(), robot.odometry.getRobotAngle());

                robot.updateRobot(false, false);
            }


        }
    }
}
