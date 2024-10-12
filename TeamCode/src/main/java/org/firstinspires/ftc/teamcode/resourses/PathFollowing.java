package org.firstinspires.ftc.teamcode.resourses;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.systems.MecanumDriveTrain;

public class PathFollowing {
  
  private ElapsedTime runtime;
  
  private double followSpeed;
  
  private PIDController xPID;
  private PIDController yPID;
  
  private MecanumDriveTrain driveTrain;
  
  private double pXConstant, pYConstant;
  private double dXConstant, dYConstant;
  
  public PathFollowing(double pX, double pY, double dX, double dY, ElapsedTime runtime, MecanumDriveTrain driveTrain){
   pXConstant = pX;
   pYConstant = pY;
   dXConstant = dX;
   dYConstant = dY;
   this.runtime = runtime;
   this.driveTrain = driveTrain;
    xPID = new PIDController(pXConstant, dXConstant, runtime);
    yPID = new PIDController(pYConstant, dYConstant, runtime);
  }
  
  public void followPath(double robotX, double robotY, double robotAngle){
    double powerS, powerF;
    
    xPID.update(robotX);
    yPID.update(robotY);
    
    driveTrain.setSpeedMultiplier(followSpeed);
  
    powerS = xPID.getPower() * Math.cos(-robotAngle) - yPID.getPower() * Math.sin(-robotAngle);
    powerF = xPID.getPower() * Math.sin(-robotAngle) + yPID.getPower() * Math.cos(-robotAngle);
    
    driveTrain.setDrivePower(powerF, powerS,);
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
