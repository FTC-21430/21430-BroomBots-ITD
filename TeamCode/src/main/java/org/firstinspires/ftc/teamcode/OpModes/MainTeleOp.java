package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Resources.InverseKinematics;
import org.firstinspires.ftc.teamcode.Resources.Utlities;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

//This is the teleop we run during competitions.
@TeleOp
public class MainTeleOp extends BaseTeleOp {
    
    
    InverseKinematics kinematics;
    boolean gp1xJoy = false;
    boolean gp2b = false;
    public double LastLoopTime;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // We multiply this by the speed to activate slowmode.
        final double slowSpeedMultiplier = 0.3;
        final double mediumSpeedMultiplier = 0.6;
        initialize();
        telemetry.setMsTransmissionInterval(10);
        robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
        kinematics = new InverseKinematics();
        waitForStart();
        while (opModeIsActive()) {
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
            
            double targetAngle = 0;
            
            if (currentArmState == armState.intake){
                
                if (gamepad1.dpad_right){
                    currentArmState = armState.spearHead;
                }

//                robot.pathFollowing.setFollowSpeed(1);
//                if (kinematics.verifyLength(robot.odometry.getRobotX(),robot.odometry.getRobotY(), 0, 20.5)) {
//                    telemetry.addLine("Kinematics passed");
//                    kinematics.calculateKinematics(robot.odometry.getRobotX(), robot.odometry.getRobotY(), 0, 20.5, 4, 0);
//
//                    robot.spampleArm.rotateElbowTo(kinematics.getElbowRotation());
//                    robot.spampleArm.rotateShoulderTo(kinematics.getArmRotation());
//                    robot.spampleArm.extendTo(kinematics.getArmLength());
//                    robot.pathFollowing.setTargetPosition(kinematics.getRobotX(),kinematics.getRobotY());
//                    robot.anglePID.setTarget(kinematics.getRobotAngle());
//                          }
                telemetry.addData("kinematics X", kinematics.getRobotX());
                telemetry.addData("kinematics Y", kinematics.getRobotY());
                telemetry.addData("kinematics arm", kinematics.getArmRotation());
                telemetry.addData("kinematics extension", kinematics.getArmLength());
                telemetry.addData("kinematics elbow", kinematics.getElbowRotation());
                
                
                
                
                robot.anglePID.update(robot.odometry.getRobotAngle());
                robot.pathFollowing.followPath(robot.odometry.getRobotX(), robot.odometry.getRobotY(), robot.odometry.getRobotAngle());
                robot.driveTrain.setDrivePower(robot.pathFollowing.getPowerF(), robot.pathFollowing.getPowerS(), robot.anglePID.getPower(), robot.odometry.getRobotAngle());


//
//
            
            } else {
                
                // resets Field Centric Driving
                
                if (gamepad1.back){
                    robot.IMUReset();
                }
                
                //  claw Positions
                
                if (gamepad1.a){
                    robot.spampleArm.setClawPosition(Claw.ClawPosition.closed);
                }
                if (gamepad1.b){
                    robot.spampleArm.setClawPosition(Claw.ClawPosition.grabInside);
                }
                if (gamepad1.y){
                    robot.spampleArm.setClawPosition(Claw.ClawPosition.grabOutside);
                }
                if (gamepad1.x){
                    robot.spampleArm.setClawPosition(Claw.ClawPosition.open);
                }

                // angles for testing only

//                if (gamepad1.a){
//                    robot.spampleArm.rotateShoulderTo(90);
//                }
//                if (gamepad1.x){
//                    robot.spampleArm.rotateShoulderTo(30);
//                }
//                if (gamepad1.y){
//                    robot.spampleArm.rotateShoulderTo(70);
//                }
//                if (gamepad1.b){
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
                if (gamepad2.y){
                    currentArmState = armState.highBasket;
                }
                if (gamepad2.b && !gp2b) {
                    if (currentArmState == armState.highChamber){
                        currentArmState = armState.scoreHighChamber;
                    } else{
                        currentArmState = armState.highChamber;
                    }
                }
                gp2b = gamepad2.b;
                if (gamepad2.a){
                    currentArmState = armState.lowChamber;
                }
                if (gamepad2.x){
                    currentArmState = armState.lowBasket;
                }
                if (gamepad2.dpad_up){
                    currentArmState = armState.dropSample;
                }
                if (gamepad2.dpad_left){
                    currentArmState = armState.grabSpecimen;
                }
                if (gamepad2.dpad_down){
                    if (currentArmState == armState.grabSpecimen){
                        currentArmState = armState.specimenIdle;
                    } else{
                        currentArmState = armState.idle;
                    }
                    
                    currentArmState = armState.idle;
                }
                if (gamepad2.dpad_right){
                    currentArmState = armState.climberReady;
                }
                if (gamepad1.dpad_left){
                    robot.spampleArm.saveShoulderTime();
                    currentArmState = armState.grabSample;
                }
                if (gamepad1.dpad_up){
                    currentArmState= armState.grabSample2;
                }
                if (gamepad1.dpad_right){
                    currentArmState = armState.spearHead;
                }
                
                if (robot.spampleArm.getTwist() <= 90 && robot.spampleArm.getTwist() >= -90){
                    robot.spampleArm.rotateTwistTo(robot.spampleArm.getTwist() + gamepad2.right_stick_x * robot.getDeltaTime() * 180);
                }
                
                
                // Sets slow mode if right bumper is pressed.
                if (gamepad1.right_bumper) {
                    robot.driveTrain.setSpeedMultiplier(slowSpeedMultiplier);
                } else if (gamepad1.right_trigger > 0.2) {
                    robot.driveTrain.setSpeedMultiplier(mediumSpeedMultiplier);
                } else {
                    robot.driveTrain.setSpeedMultiplier(1);
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
                targetAngle = Utlities.wrap(robot.anglePID.getTarget() + (-gamepad1.right_stick_x * robot.maxTurnDegPerSecond * robot.getDeltaTime() * robot.driveTrain.getSpeedMultiplier()));
                
                if (Math.abs(gamepad1.left_stick_x) < 0.1 == false && gp1xJoy == true) {
                    targetAngle = robot.odometry.getRobotAngle();
                }
                
                // These call functions and pass the relevant parameters
                robot.anglePID.setTarget(targetAngle);
//                robot.anglePID.update(robot.odometry.getRobotAngle());
                robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.anglePID.getPower(), robot.odometry.getRobotAngle());
                
            }
            
            
            updateState();
            
            
            robot.spampleArm.updateArm();
            
            // the old input for the left stick x axis for gamepad 1,
            // updated at the end of the loop so the turning logic works :)
            gp1xJoy = gamepad1.left_stick_x > 0.1;

            
            
            // Telemetry for testing/debug purposes

            telemetry.addData("arm extension", robot.spampleArm.getArmExtension());

            telemetry.addData("current Speed", robot.driveTrain.getSpeedMultiplier());
            telemetry.addData("currentState", currentArmState);
            telemetry.addData("Robot Angle",robot.odometry.getRobotAngle());
            telemetry.addData("Target Angle",targetAngle);
            telemetry.addData("arm angle", robot.spampleArm.getArmAngle());
            telemetry.addData("Robot X", robot.odometry.getRobotX());
            telemetry.addData("Robot Y", robot.odometry.getRobotY());
            telemetry.addData("Potemeter", robot.spampleArm.getArmAngle());
            double currentlooptime= getRuntime();
            telemetry.addData("Runtime", currentlooptime-LastLoopTime);
            LastLoopTime=currentlooptime;
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
