package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//This class is used to track the robots position while the op-mode is running.
public class OdometryOTOS {
  // The IMU(Inertial Measurement Unit) tracks the angles of the robot
  SparkFunOTOS OTOS;
  
  //where the center of the robot is now
  private double robotX, robotY; // in inches
  private double robotAngle; // in degrees
  
  /**
   * the contructor for this class, used to set some variables
   * @param initX
   * @param initY
   * @param robotAngle
   * @param telemetry
   */
  public OdometryOTOS(double initX, double initY, double robotAngle, Telemetry telemetry, HardwareMap hardwareMap){
    OTOS = hardwareMap.get(SparkFunOTOS.class, "OTOS");
    configureOtos();
    robotX = initX;
    robotY = initY;
    this.robotAngle = robotAngle;
    overridePosition(robotX,robotY,robotAngle);
  }
  
  // this does all of the math to recalculate where to robot is.
  public void updateOdometry() {
    SparkFunOTOS.Pose2D pos = OTOS.getPosition();
    robotX = pos.x;
    robotY = pos.y;
    robotAngle = pos.h;
    
  }
  
  /**
   * for when you need to override the tracking data
   * @param x new input x in inches
   * @param y new input y in inches
   * @param angle new input angle in radians
   */
  public void overridePosition(double x, double y, double angle){
    SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, angle);
    OTOS.setPosition(currentPosition);
  }
  public double getRobotAngle(){return robotAngle;}
  
  public double getRobotX(){return robotX;}
  
  public double getRobotY(){return robotY;}
  
  private void configureOtos() {
    OTOS.setLinearUnit(DistanceUnit.INCH);
    OTOS.setAngularUnit(AngleUnit.DEGREES);
    
    SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -4.6, 90);
    OTOS.setOffset(offset);
    
    // Here we can set the linear and angular scalars, which can compensate for
    // scaling issues with the sensor measurements. Note that as of firmware
    // version 1.0, these values will be lost after a power cycle, so you will
    // need to set them each time you power up the sensor. They can be any value
    // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
    // first set both scalars to 1.0, then calibrate the angular scalar, then
    // the linear scalar. To calibrate the angular scalar, spin the robot by
    // multiple rotations (eg. 10) to get a precise error, then set the scalar
    // to the inverse of the error. Remember that the angle wraps from -180 to
    // 180 degrees, so for example, if after 10 rotations counterclockwise
    // (positive rotation), the sensor reports -15 degrees, the required scalar
    // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
    // robot a known distance and measure the error; do this multiple times at
    // multiple speeds to get an average, then set the linear scalar to the
    // inverse of the error. For example, if you move the robot 100 inches and
    // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
    OTOS.setLinearScalar(1.0);
    OTOS.setAngularScalar(1.0);

    OTOS.calibrateImu();
    
    // Reset the tracking algorithm - this resets the position to the origin,
    // but can also be used to recover from some rare tracking errors
    OTOS.resetTracking();
    
    SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
    OTOS.setPosition(currentPosition);
    
    // Get the hardware and firmware version
    SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
    SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
    OTOS.getVersionInfo(hwVersion, fwVersion);
    
  }
  
}