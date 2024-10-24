package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.teamcode.Resources.AprilTagSystem;
import org.firstinspires.ftc.teamcode.Resources.Utlities;

@Disabled
@TeleOp
public class AprilTagTest extends BaseTeleOp {
  private AprilTagSystem aprilTagSystem;
  
  @Override
  public void runOpMode() throws InterruptedException {
    
    aprilTagSystem = new AprilTagSystem(hardwareMap);
    
    initialize();
    
    waitForStart();
    while (opModeIsActive()) {
      //robot.setIsOpModeActive(opModeIsActive());
      robot.odometry.updateOdometry();
      aprilTagSystem.findAprilTags(robot.odometry.getRobotX(), robot.odometry.getRobotY());
      if (aprilTagSystem.hasDetection()) {
        telemetry.addData("robotX: ", aprilTagSystem.getRobotX());
        telemetry.addData("robotY: ", aprilTagSystem.getRobotY());
        telemetry.addData("robotYaw", aprilTagSystem.getRobotYaw());
      }
      telemetry.update();
      
    }
  }
}
