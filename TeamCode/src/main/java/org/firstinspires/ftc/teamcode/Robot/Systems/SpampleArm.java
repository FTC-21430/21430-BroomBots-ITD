package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//This class is the code foundations for making the robot's arm move.
public class SpampleArm {
    
    private ElapsedTime runtime = null;
    private double elbowAngleOffset = 1029;
    private double shoulderAngleOffset;
    
    //Arm sensors
    public AnalogInput armPotentiometer = null;
    
    //Actuators for the arm
    
    enum armPositions{
        highBasket,
        lowBasket,
        highChamber,
        lowChamber,
        dropOff,
        idle,
        
    }
    
    public DcMotor shoulderMotor;
    DcMotor linearSlideMotor;
    ServoPlus elbowServo;
    
    double elbowTimer =0.0;
    
    ServoPlus twistServo;
    Claw claw;

    double targetExtension = 0;
    
    //TODO: replace with correct value; calibrated for 312 RPM motor
    //the motor will not turn correctly without these values right.
    //Constants for the shoulder
    final double shoulderPulsesPerRevolution = 8011.117;
    final double shoulderTicksPerDegrees = shoulderPulsesPerRevolution / 360;
    
    //Constants for the linear slide
    final double linearSlidePulsesPerRevolution = 1223.08;
    // multiplied by two because of the cascade rigging
    final double linearSlideRevPerInch = 1/(4.724*2);
    final double linearSlideTicksPerInch = linearSlidePulsesPerRevolution * linearSlideRevPerInch;
    final double linearSlideMaxExtension = 19.5;
    
    // used to correct the error caused in the slide by the rotation of the shoulder.
    final double shoulderRotationToSlide = -linearSlidePulsesPerRevolution/shoulderPulsesPerRevolution;

    /**
     * Arm constructor
     * @param hardwareMap Robot hardware map
     */
    public SpampleArm (HardwareMap hardwareMap, ElapsedTime runtime){
        //Mapping/initializing motors
        shoulderMotor = hardwareMap.get(DcMotor.class,"shoulderMotor");
        shoulderMotor.setTargetPosition(0);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // you need to set how fast the motor moves before it will move at all.
        shoulderMotor.setPower(1);
        shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideMotor = hardwareMap.get(DcMotor.class,"linearSlideMotor");
        linearSlideMotor.setTargetPosition(0);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // you need to set how fast the motor moves before it will move at all.
        linearSlideMotor.setPower(1);

        //Mapping/initializing servos
        elbowServo = new ServoPlus(hardwareMap.get(Servo.class,"elbowServo"),
                1650,0,1650);

        twistServo = new ServoPlus(hardwareMap.get(Servo.class,"twistServo"),
                260,0,260);

        claw = new Claw(hardwareMap);
        
        armPotentiometer = hardwareMap.get(AnalogInput.class, "shoulderAngleP");
        shoulderAngleOffset = getArmAngle();
        
        this.runtime = runtime;
    }

    
    
    public double getArmAngle(){
        return -143.12* armPotentiometer.getVoltage()+248.72;
    }
    
    /**
     * Controls the shoulder motor
     * @param angle Angle for shoulder in degrees
     */
    public void rotateShoulderTo (double angle){
        
        // I tried to do some fancy calibration and stuff but it did not work :(
//        double correctedAngle = angle - (6.33 + 9.66E-03 * angle + -1.12E-03 * Math.pow(angle, 2));
        double correctedAngle;
        if (angle <= 30){
            // to acount for the error of 7.2 degrees when picking up from angles less than 30 degrees
            correctedAngle = angle + 7.2;
        }
        
        
        else{
            correctedAngle = angle;
        }
        
        
        //this ensures that the rotation of the robot's arm is never past the mechanical constraints of the robot
        if (angle < 8.5) {
            angle = 8.5;
        }
        if (angle > 169.5) {
            angle = 169.5;
        }
        
        shoulderMotor.setTargetPosition((int) ((correctedAngle- shoulderAngleOffset) * shoulderTicksPerDegrees));
    }
    
    /**
     * because the shoulder could be moving between setter calls of the linear slide, we have to update is constantly to correct.
     */
    public void updateSlide(){
        linearSlideMotor.setTargetPosition((int) ((targetExtension * linearSlideTicksPerInch) + (shoulderMotor.getCurrentPosition() * shoulderRotationToSlide)));
    }
    
    /**
     * Controls the linear slide
     * @param inches Distance for extension in inches
     */
    public void extendTo (double inches){
        if (inches < 0){
            inches = 0;
        }
        if (inches > linearSlideMaxExtension) {
            inches = linearSlideMaxExtension;
        }
        targetExtension = inches;
    }

    /**
     * Controls the elbow motor
     * @param angle Angle for elbow in degrees
     */

    public void rotateElbowTo (double angle){
        elbowServo.setServoPos(angle+elbowAngleOffset);
        elbowTimer = runtime.milliseconds();
    }

    /**
     * Controls the twist of the twist
     * @param angle Angle for twist in degrees
     */
    public void rotateTwistTo (double angle){
        twistServo.setServoPos(angle+15);
    }

    /**
     * Controls the four positions of the claw
     * @param position Must be one of the values from {@link org.firstinspires.ftc.teamcode.Robot.Systems.Claw.ClawPosition}
     */
    public void setClawPosition (Claw.ClawPosition position){
        claw.setPosition(position);
    }



    // TODO Functions:
    /*
  X Dropoff             -D-pad up
  X Grab Specimen       -D-pad left
  X Idle                -D-pad down
  X Extension Offset    -Joystick left
    Twist (beware)      -Joystick right
  X High Basket         -Y
  X Low Basket          -X
  X High Chamber        -B
  X Low Chamber         -A

     */
    //shoulder
    //extensor
    //elbow
    //twist
    //pinchy

    //High Basket
    //fix variables
    
   
    
    
    
    
    

    //TODO: tune the tick values to be the most optimized for our needs.
    public boolean shoulderAtPosition(){
        double shoulderErrorThreshold = 5; // in degrees
        // returns true if where we are is within 20 ticks of where we want to be.
        return Math.abs(shoulderMotor.getCurrentPosition() - shoulderMotor.getTargetPosition()) < shoulderErrorThreshold * shoulderTicksPerDegrees;
    }
    public boolean extensionAtPosition(){
        double extensionTargetErrorThreshold = 1; // in inches
        // returns true if where we are is within 20 ticks of where we want to be.
        return Math.abs(linearSlideMotor.getCurrentPosition() - linearSlideMotor.getTargetPosition()) < extensionTargetErrorThreshold*linearSlideTicksPerInch;
    }
    public boolean elbowAtPosition(){
        double elbowTimeS = 5;
        return runtime.milliseconds()-elbowTimer > elbowTimeS *1000;
    }

}