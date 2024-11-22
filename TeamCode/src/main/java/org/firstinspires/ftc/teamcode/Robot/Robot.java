package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Resources.Odometry;
import org.firstinspires.ftc.teamcode.Robot.Systems.MecanumDriveTrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;

@Config
public class Robot {
  
  
  //used for how fast the turning input is used.
  // the number for maxTurnDegPerSecond is how much the robot can turn for one degree
  public static double maxTurnDegPerSecond = 500;
  public static double pCon = 0.017;
  public static double dCon = 0;
  
  private double drive;
  private double slide;
  private double turn;
  //TODO Make more permanent system to detect turning
  public boolean turningBoolean;
  public MecanumDriveTrain driveTrain;
  public Odometry odometry;
  
  private ElapsedTime runtime = new ElapsedTime();
  //TODO Tune the pConstant and d Constant numbers, these are place holders.
  public PIDController anglePID = new PIDController(pCon, dCon, runtime);
  
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
 
  
  public void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle) {
    driveTrain = new MecanumDriveTrain(hardwareMap, telemetry);
    odometry = new Odometry(robotX, robotY, robotAngle, telemetry, hardwareMap);
    //TODO These numbers are placeholders
    pathFollowing = new PathFollowing(1, 1, 1, 1, runtime);
  }
  
  // you call this function in a main auto opMode to make the robot move somewhere.
  // This is the foundation that every robot should need but you should more season specific things in the bot class.
  public void autoMoveTo(double targetX, double targetY, double robotAngle, double targetCircle) {
    while (distanceCircle(targetX, targetY) > targetCircle && opModeActive) {
      //put all control things that teleop has in here
      pathFollowing.followPath(odometry.getRobotX(), odometry.getRobotY(), odometry.getRobotAngle());
      driveTrain.setDrivePower(pathFollowing.getPowerF(), pathFollowing.getPowerS(), anglePID.getPower(), odometry.getRobotAngle());
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
    odometry.IMUReset();
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
