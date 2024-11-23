package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Resources.InverseKinematics;
import org.firstinspires.ftc.teamcode.Resources.Utlities;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

//This is the teleop we run during competitions.
@TeleOp
public class MainTeleOp extends BaseTeleOp {
    
    SpampleArm spampleArm;
    InverseKinematics kinematics;
    
    
    @Override
    public void runOpMode() throws InterruptedException {
        // We multiply this by the speed to activate slowmode.
        final double slowSpeedMultiplier = 0.40;
        initialize();
        spampleArm = new SpampleArm(hardwareMap);
        kinematics = new InverseKinematics();
        waitForStart();
        while (opModeIsActive()) {
            
            if (gamepad1.a){
                spampleArm.setClawPosition(Claw.ClawPosition.closed);
            }
            if (gamepad1.b){
                spampleArm.setClawPosition(Claw.ClawPosition.grabInside);
            }
            if (gamepad1.y){
                spampleArm.setClawPosition(Claw.ClawPosition.grabOutside);
            }
            if (gamepad1.x){
                spampleArm.setClawPosition(Claw.ClawPosition.open);
            }
            
            if (gamepad1.dpad_up){
                if (kinematics.verifyLength(0,0,12,24)){
                    kinematics.calculateKinematics(0,0,12,24,4,0);
                    spampleArm.rotateShoulderTo(30);
                    spampleArm.extendTo(kinematics.getArmLength());
                    spampleArm.rotateElbowTo(kinematics.getElbowRotation());
                    robot.anglePID.setTarget(kinematics.getRobotAngle());
                    telemetry.addData("robotRotation", kinematics.getRobotAngle());
                    telemetry.addData("armPivot", kinematics.getArmRotation());
                    telemetry.addData("elbowRotation", kinematics.getElbowRotation());
                    telemetry.addData("armExtension", kinematics.getArmLength());
                }
            }
            if (gamepad1.dpad_down){
                if (kinematics.verifyLength(0,0,12,24)){
                    kinematics.calculateKinematics(0,0,12,24,4,0);
                    spampleArm.rotateShoulderTo(kinematics.getArmRotation());
                    spampleArm.extendTo(kinematics.getArmLength());
                    spampleArm.rotateElbowTo(kinematics.getElbowRotation());
                    robot.anglePID.setTarget(kinematics.getRobotAngle());
                    telemetry.addData("robotRotation", kinematics.getRobotAngle());
                    telemetry.addData("armPivot", kinematics.getArmRotation());
                    telemetry.addData("elbowRotation", kinematics.getElbowRotation());
                    telemetry.addData("armExtension", kinematics.getArmLength());
                }
            }
            
            
            
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
                
                
               
                
//                telemetry.addData("arm angle", spampleArm.getArmAngle());
                
                
                spampleArm.updateSlide();
                
                telemetry.update();
            
            
            
            // Telemetry for testing purposes
            telemetry.addData("Robot Angle",robot.odometry.getRobotAngle());
            telemetry.addData("Target Angle",targetAngle);
            telemetry.addData("arm angle", spampleArm.getArmAngle());
            telemetry.update();

            }
        }
    }
}
