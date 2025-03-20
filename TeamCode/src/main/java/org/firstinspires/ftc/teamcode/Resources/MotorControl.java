package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.robotcore.util.ElapsedTime;

/// this class is designed for smoother control
///  of movement components by using motion profiling concepts.
public class MotorControl {

    // the max speed per second you can accelerate the
    private double maxAcceleration = 1.0;
    //the max speed of the motor, between 0.0-1.0
    private double maxSpeed = 1.0;
    private ElapsedTime runtime = null;


    public MotorControl(double maxAccel, double maxSpeed, ElapsedTime runtime){
        // set motion profiling constants
        maxAcceleration = maxAccel;
        this.maxSpeed = maxSpeed;
        this.runtime = runtime;
    }

    public double calculateMotorPower(double currentPosition){
        
    }

    public void startMove(double currentPosition, double targetPosition){
        runtime.reset();
    }

    public void updateConstants(double maxAccel, double maxSpeed){
        maxAcceleration = maxAccel;
        this.maxSpeed = maxSpeed;
    }
}
