package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Resources.InverseKinematics;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SampleCamera;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@TeleOp
public class SampleDetectionTesting extends BaseTeleOp {
    private SampleCamera sampleCamera;

    private InverseKinematics kinematics;


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
    private double hoverZ = 4;
    private double grabZ = 0.0;

    double targetAngle = 0;
    private double alignmentTime = 0.6;

    private double loweringTime = 0.5;

    private double grabbingTime = 0.3;

    private double startingAngle = 0;

    private boolean aligning = false;
    private boolean lowering = false;
    private boolean grabbing = false;
    private double autoPickupTimer = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {

        initialize(true, false);

        sampleCamera = new SampleCamera(hardwareMap, telemetry);

        kinematics = new InverseKinematics();

//        sampleCamera.findYellowSample();

        waitForStart();

        while(opModeIsActive()) {


            if (gamepad1.cross) {
                robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
            }
            if (gamepad1.square) {
                robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
            }
            if (gamepad1.circle) {
                robot.spampleArm.setClawPosition(Claw.ClawPosition.grabInside);
            }


            robot.updateLoopTime();

            // get and update functions
            robot.odometry.updateOdometry();
            // These call functions and pass the relevant parameters


//            if (robot.spampleArm.currentArmState == SpampleArm.armState.pictureTake) {
//                sampleCamera.startDetection();
//            } else {
//                sampleCamera.stopDetection();
//            }

            if (gamepad1.right_bumper && !grabbingSample) {
                robot.spampleArm.currentArmState = SpampleArm.armState.idle;
                robot.setTurnPIntake(false);
            }

            if (gamepad1.left_bumper) {
                grabbingSample = false;
                lookingForSample = false;
                sampleCamera.stopDetection();
                robot.spampleArm.currentArmState = SpampleArm.armState.pictureTake;
                robot.anglePID.setTarget(0);
                robot.setTurnPIntake(false);
            }
            if (gamepad1.dpad_down) {
                lookingForSample = true;
                sampleCamera.findYellowSample();
                startedLookingTime = runtime.time();
            }
            if (gamepad1.dpad_left){
                lookingForSample = true;
                sampleCamera.findRedSample();
                startedLookingTime = runtime.time();
            }
            if (gamepad1.dpad_right){
                lookingForSample = true;
                sampleCamera.findBlueSample();
                startedLookingTime = runtime.time();
            }

            if (lookingForSample) {
                if (runtime.time() > startedLookingTime + searchTimeout) {
                    lookingForSample = false;
                    gamepad1.rumble(700);
                } else if (sampleCamera.didWeFindOne()) {
                    lookingForSample = false;

                    sampleCamera.findCameraPosRelativePosition(robot.spampleArm.getArmAngle(), robot.spampleArm.getArmExtension(), 0);

                    sampleCamera.stopDetection();

                    foundX = sampleCamera.getSampleX();
                    foundY = sampleCamera.getSampleY();
                    foundYaw = sampleCamera.getSampleYaw();

                    telemetry.addLine("SampleX: " + foundX);
                    telemetry.addLine("SampleY: " + foundY);
                    telemetry.addLine("SampleYaw: " + foundYaw);
                    telemetry.addLine("------------");
                    telemetry.addLine("camera X: " + sampleCamera.cameraXRobot);
                    telemetry.addLine("camera Y: " + sampleCamera.cameraYRobot);
                    telemetry.addLine("camera Z: " + sampleCamera.cameraZRobot);

                    if (kinematics.calculateKinematics(0, 0, foundX, foundY, grabZ, foundYaw, robot.odometry.getRobotAngle())) {

                        gamepad1.rumble(0.0, 0.5, 1000);

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
                        gamepad1.rumble(2400);
                    }
                    telemetry.update();
                }
            }


            if (!grabbingSample && !lookingForSample) {
                targetAngle = (robot.anglePID.getTarget() + (-gamepad1.right_stick_x * robot.maxTurnDegPerSecond * robot.getDeltaTime() * robot.driveTrain.getSpeedMultiplier()));

                robot.driveTrain.setSpeedMultiplier(0.8);

                // These call functions and pass the relevant parameters
                robot.anglePID.setTarget(targetAngle);
                robot.anglePID.update(robot.odometry.getRobotAngle());

                robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.anglePID.getPower(), robot.odometry.getRobotAngle());

                robot.updateRobot(false, false, false);

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

            robot.updateRobot(false, false, false);

        }
        }
    }
}
