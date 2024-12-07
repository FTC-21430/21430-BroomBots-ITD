package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Resources.AprilTagSystem;
import org.firstinspires.ftc.teamcode.Resources.OdometryOTOS;
import org.firstinspires.ftc.teamcode.Robot.Systems.MecanumDriveTrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;
import org.firstinspires.ftc.teamcode.Robot.Systems.SpampleArm;

@Config
public class Robot {
  
  
  //used for how fast the turning input is used.
  // the number for maxTurnDegPerSecond is how much the robot can turn for one degree
  public static double maxTurnDegPerSecond = 280;
  public static double pCon = 0.025;
  public static double dCon = 0;
  
  private double drive;
  private double slide;
  private double turn;
  //TODO Make more permanent system to detect turning
  public boolean turningBoolean;
  public MecanumDriveTrain driveTrain;
  public OdometryOTOS odometry;
  

  
  private ElapsedTime runtime = new ElapsedTime();
  //TODO Tune the pConstant and d Constant numbers, these are place holders.
  public PIDController anglePID = new PIDController(pCon, 0, dCon, runtime);
  
  public SpampleArm spampleArm;
  FtcDashboard dashboard;
  private double robotHeading;
  private double lastTimeAngle;
  private boolean CurrentAlign = true;
  private boolean DriverOrientationDriveMode = true;
  
  public boolean opModeActive;
  
  public double derivativeConstantAngle;
  public double proportionalConstantAngle;
  public Telemetry telemetry;
  private boolean resettingImu = false;
  private double AutoStartAngle = 0;
  
  private double currentLoopTime, previousLoopTime;
  private double lastErrorAngle;
  private boolean IsProgramAutonomous;
  public PathFollowing pathFollowing;
 
  public AprilTagSystem aprilTags;
  public void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle) {
    
    driveTrain = new MecanumDriveTrain(hardwareMap, telemetry);

    odometry = new OdometryOTOS(robotX, robotY, robotAngle, telemetry, hardwareMap);
    //TODO These numbers are placeholders
    pathFollowing = new PathFollowing(0.12, 0.17, 0.01, 0.01, runtime);
    spampleArm = new SpampleArm(hardwareMap, runtime);
    aprilTags = new AprilTagSystem(hardwareMap);
  }
  
  // you call this function in a main auto opMode to make the robot move somewhere.
  // This is the foundation that every robot should need but you should more season specific things in the bot class.
  public void autoMoveTo(double targetX, double targetY, double robotAngle, double targetCircle) {
    while (distanceCircle(targetX, targetY) > targetCircle && opModeActive) {
      pathFollowing.setTargetPosition(targetX,targetY);
      anglePID.setTarget(robotAngle);
      //put all control things that teleop has in here
      anglePID.update(odometry.getRobotAngle());
      pathFollowing.followPath(odometry.getRobotX(), odometry.getRobotY(), odometry.getRobotAngle());
      driveTrain.setDrivePower(pathFollowing.getPowerF(), pathFollowing.getPowerS(), anglePID.getPower(), odometry.getRobotAngle());
      
      spampleArm.updateArm();
    }
  }
  
  public double getDeltaTime() {
    double deltaTime;
    currentLoopTime = runtime.seconds();
    deltaTime = currentLoopTime - previousLoopTime;
    previousLoopTime = currentLoopTime;
    return deltaTime;
  }
  
  public void IMUReset() {
    odometry.overridePosition(odometry.getRobotX(),odometry.getRobotY(),0);
    anglePID.setTarget(0);
  }
  
 
  
  // This is the equation used to convert from radians to degrees
  public double radiansToDegrees(double radians) {
    return radians * (180 / Math.PI);
  }
  
  // This is the equation used to convert from degrees to radians
  public double degreesToRadians(double degrees) {
    return degrees * (Math.PI / 180);
  }
  
  public void turnUpdate() {
    if (resettingImu) {
      anglePID.setTarget(odometry.getRobotAngle());
      anglePID.update(odometry.getRobotAngle());
      return;
    }
  }
    
    //setter for OpModeActive
    public void setIsOpModeActive ( boolean isOpModeActive){
      this.opModeActive = isOpModeActive;
    }
  
    public double distanceCircle(double x, double y){
      return (Math.sqrt((x - odometry.getRobotX()) * (x - odometry.getRobotX()) + (y - odometry.getRobotY()) * (y - odometry.getRobotY())));
    }
  }
