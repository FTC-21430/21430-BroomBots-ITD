package org.firstinspires.ftc.teamcode.resourses;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.systems.MecanumDriveTrain;

public class PathFollowing {
  
  private ElapsedTime runtime;
  
  private double followSpeed = 1;
  
  private PIDController xPID;
  private PIDController yPID;
  
  private double pXConstant, pYConstant;
  private double dXConstant, dYConstant;
  
  double powerS, powerF;
  
  public PathFollowing(double pX, double pY, double dX, double dY, ElapsedTime runtime){
   pXConstant = pX;
   pYConstant = pY;
   dXConstant = dX;
   dYConstant = dY;
   this.runtime = runtime;
  
    xPID = new PIDController(pXConstant, dXConstant, runtime);
    yPID = new PIDController(pYConstant, dYConstant, runtime);
  }
  
 
  
  public void followPath(double robotX, double robotY, double robotAngle){
    xPID.update(robotX);
    yPID.update(robotY);
    powerS = xPID.getPower() * Math.cos(Math.toRadians(-robotAngle)) - yPID.getPower() * Math.sin(Math.toRadians(-robotAngle)) * followSpeed;
    powerF = xPID.getPower() * Math.sin(Math.toRadians(-robotAngle)) + yPID.getPower() * Math.cos(Math.toRadians(-robotAngle)) * followSpeed;
  }
  
  public double getPowerS(){
    return powerS;
  }
  
  public double getPowerF(){
    return powerF;
  }
  
  public void setTargetPosition(double x, double y){
    xPID.setTarget(x);
    yPID.setTarget(y);
  }
  
  public void setFollowSpeed(double speed){
    followSpeed = speed;
  }
  
  public double getFollowSpeed(){ return followSpeed; }
  
}
