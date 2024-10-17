package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Robot.systems.MecanumDriveTrain;
import android.view.View;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.resourses.Odometry;
import org.firstinspires.ftc.teamcode.resourses.PathFollowing;

@Config
public class Robot {

    private double TargetAngle = 0;
    private double lastTimeAngle;

    private double drive;
    private double slide;
    private double turn;

    public MecanumDriveTrain driveTrain;
    public Odometry odometry;
  
    private ElapsedTime runtime = new ElapsedTime();
    FtcDashboard dashboard;
    private boolean CurrentAlign = true;
    private boolean DriverOrientationDriveMode = true;
    
    public boolean opModeActive;
    
    public double derivativeConstantAngle;
    public double proportionalConstantAngle;
    public Telemetry telemetry = null;
    
    private double lastErrorAngle;
    private boolean IsProgramAutonomous;
    public PathFollowing pathFollowing;
    
    public void init(HardwareMap hardwareMap, double robotX, double robotY, double robotAngle){
        driveTrain = new MecanumDriveTrain(hardwareMap);
        odometry = new Odometry(robotX,robotY,robotAngle, telemetry);
        pathFollowing = new PathFollowing(1,1,1,1,runtime);
    }

    // you call this function in a main auto opMode to make the robot move somewhere.
    // This is the foundation that every robot should need but you should more season specific things in the bot class.
    public void autoMoveTo(double targetX, double targetY, double robotAngle, double targetCircle){
        while(distanceCircle(targetX, targetY) > targetCircle &&opModeActive){
            //put all control things that teleop has in here
            pathFollowing.followPath(odometry.getRobotX(),odometry.getRobotY(),odometry.getRobotAngle());
            driveTrain.setDrivePower(pathFollowing.getPowerF(), pathFollowing.getPowerS(), anglePID.getPower(), odometry.getRobotAngle());
        }
    }
    
    
    public void ProportionalFeedbackControl() {
        double currentTime = runtime.time();
        double derivativeAngle;
        double error = 0;
        if (odometry.isResetingIMU())
            return;
        telemetry.addData("target", TargetAngle);
        error = Wrap((TargetAngle/180)*Math.PI - odometry.getRobotAngle())*180/Math.PI;
        derivativeAngle = (error - lastErrorAngle)/(currentTime - lastTimeAngle);

        TargetAngle = (odometry.getRobotAngle() * 180 / Math.PI);

        telemetry.addData("ERROR", error);
        telemetry.addData("BEFORE", turn);
        turn -= error * proportionalConstantAngle + (derivativeConstantAngle * derivativeAngle);
        telemetry.addData("AFTER", turn);
        lastTimeAngle = currentTime;
        lastErrorAngle = error;
    }
    
    
    public void IMUReset() {
        TargetAngle = 0;
        odometry.IMUReset();
    }
    
    
    //setter for OpModeActive
    public void setIsOpModeActive(boolean isOpModeActive){
        this.opModeActive=isOpModeActive;
    }
    
    public double distanceCircle(double x, double y) {
        return (Math.sqrt((x - odometry.getRobotX()) * (x - odometry.getRobotX())
                + (y - odometry.getRobotY()) * (y - odometry.getRobotY())));
    }
    
    
    double Wrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}