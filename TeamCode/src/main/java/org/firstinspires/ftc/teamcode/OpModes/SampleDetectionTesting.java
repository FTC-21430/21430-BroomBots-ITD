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

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(true);

        sampleCamera = new SampleCamera(hardwareMap, telemetry);

        kinematics = new InverseKinematics();

//        sampleCamera.findYellowSample();

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.dpad_left){
                sampleCamera.findBlueSample();
            }
            if (gamepad1.dpad_right){
                sampleCamera.findRedSample();
            }
            if (gamepad1.dpad_left){
                sampleCamera.findYellowSample();
            }

            if (gamepad1.cross){
                robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
            }
            if (gamepad1.square){
                robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
            }
            if (gamepad1.circle){
                robot.spampleArm.setClawPosition(Claw.ClawPosition.grabInside);
            }



            robot.updateLoopTime();

            // get and update functions
            robot.odometry.updateOdometry();
            // These call functions and pass the relevant parameters

            if (gamepad1.right_trigger > 0.7) {
                robot.setTurnPIntake(true);
                robot.spampleArm.currentArmState = SpampleArm.armState.test;
                robot.anglePID.setTarget(target_r_rot);
                robot.spampleArm.rotateElbowTo(elbow);
                robot.spampleArm.rotateTwistTo(twist);
                robot.spampleArm.rotateShoulderTo(shoulder_rot);
                robot.spampleArm.extendTo(extension);
            } else if(gamepad1.left_trigger >0.6){
                robot.spampleArm.currentArmState = SpampleArm.armState.pictureTake;
                robot.setTurnPIntake(false);
            }else{
                robot.spampleArm.currentArmState = SpampleArm.armState.idle;
                robot.setTurnPIntake(false);
            }

            if (robot.spampleArm.currentArmState == SpampleArm.armState.pictureTake) {
                sampleCamera.startDetection();
            } else {
                sampleCamera.stopDetection();
            }


            double z_target;
            if (gamepad1.right_bumper){
                z_target = 0.25;
            }else{
                z_target = 3.5;
            }


            if (sampleCamera.didWeFindOne()) {
                sampleCamera.findCameraPosRelativePosition(30.0, 0);


                foundX = sampleCamera.getSampleX();
                foundY = sampleCamera.getSampleY();
                foundYaw = sampleCamera.getSampleYaw();



            }
            if (kinematics.calculateKinematics(0, 0, sampleCamera.getSampleX(), sampleCamera.getSampleY(), z_target, 0)) {


                telemetry.addLine("arm extension: " + kinematics.getArmExtension());
                telemetry.addLine("shoulder rotation: " + kinematics.getArmRotation());
                telemetry.addLine("elbow rotation: " + kinematics.getElbowRotation());
                telemetry.addLine("robot rotation: " + kinematics.getRobotAngle());
                telemetry.addLine("twist rotation: " + kinematics.getTwist());
                telemetry.addLine("-------------------");


                target_r_rot = kinematics.getRobotAngle();
                shoulder_rot = kinematics.getArmRotation();
                elbow = kinematics.getElbowRotation();
                extension = kinematics.getArmExtension();
                twist = kinematics.getTwist();
            }

            telemetry.addLine("Found a Sample!");
            telemetry.addLine("SampleX: " + foundX);
            telemetry.addLine("SampleY" + foundY);
            telemetry.addLine("SampleYaw" + foundYaw);
            telemetry.addLine("------------");

            robot.driveTrain.setSpeedMultiplier(0.8);
            robot.anglePID.update(robot.odometry.getRobotAngle());

            telemetry.addData("angle PID output", robot.anglePID.getPower());
            robot.driveTrain.setDrivePower(0, 0, robot.anglePID.getPower(), robot.odometry.getRobotAngle());

            robot.updateRobot(false, false);
            telemetry.update();
        }
    }

}
