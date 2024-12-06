package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.PIDController;

//This class is the code foundations for making the robot's arm move.
@Config
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
    
    
    public PIDController shoulderPID;
    
    public DcMotor shoulderMotor;
    DcMotor linearSlideMotor;
    ServoPlus elbowServo;
    ServoPlus twistServo;
    Claw claw;
// the robot pitch from the field floor... odd...
    
    double robotTilt = 1.0;
    
    double targetExtension = 0;
    
    //TODO: replace with correct value; calibrated for 312 RPM motor
    //the motor will not turn correctly without these values right.
    //Constants for the shoulder
    final double shoulderPulsesPerRevolution = 8011.117;
    final double shoulderTicksPerDegrees = shoulderPulsesPerRevolution / 360;
    
    //Constants for the linear slide
    final double linearSlidePulsesPerRevolution = 1223.08;
    // multiplied by two because of the cascade rigging
    final double linearSlideRevPerInch = 1/(4.725*2);
    final double linearSlideTicksPerInch = linearSlidePulsesPerRevolution * linearSlideRevPerInch;
    final double linearSlideMaxExtension = 19.625984;
    
    // used to correct the error caused in the slide by the rotation of the shoulder.
    final double shoulderRotationToSlide = -linearSlidePulsesPerRevolution/shoulderPulsesPerRevolution;

    public static double pConstant = 0.028;
    public static double iConstant = 0.06;
    public static double dConstant =0.0005;
    private double elbowTimer = 0.0;

    private double shoulderTimer = 0.0;
    
    /**
     * Arm constructor
     * @param hardwareMap Robot hardware map
     */
    public SpampleArm (HardwareMap hardwareMap, ElapsedTime runtime){
        
        
        shoulderPID = new PIDController(pConstant, iConstant,dConstant, new ElapsedTime());
        shoulderPID.setTarget(90);
        
        //Mapping/initializing motors
        shoulderMotor = hardwareMap.get(DcMotor.class,"shoulderMotor");
        
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // you need to set how fast the motor moves before it will move at all.
        
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
        return 38.412 * Math.pow(armPotentiometer.getVoltage(),2) - 232.78 * armPotentiometer.getVoltage() + 299.5 - robotTilt;
    }

    public double getArmExtension(){
        return linearSlideMotor.getCurrentPosition() / linearSlideTicksPerInch;
    }

    /**
     * Controls the shoulder motor
     * @param angle Angle for shoulder in degrees
     */
    public void rotateShoulderTo (double angle){
        //this ensures that the rotation of the robot's arm is never past the mechanical constraints of the robot
        if (angle < 8.5) {
            angle = 8.5;
        }
        if (angle > 178) {
            angle = 178;
        }
        shoulderPID.setTarget(angle);
    }
    
    /**
     * because the shoulder could be moving between setter calls of the linear slide, we have to update is constantly to correct.
     */
    public void updateArm(){

        if (getArmExtension() >= 7){
            shoulderPID.setIntegralMode(false);
        }else{
            shoulderPID.setIntegralMode(true);
        }

//        shoulderPID.updateConstants(pConstant, iConstant, dConstant);
        shoulderPID.update(getArmAngle());
        shoulderMotor.setPower(shoulderPID.getPower() + Math.cos(getArmAngle() *Math.PI/180)*0.12);
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
        // values to ensure the twist goes where we need it to, then rotated by 90 degrees
        twistServo.setServoPos(angle+16+ 90);
    }

    /**
     * Controls the four positions of the claw
     * @param position Must be one of the values from {@link org.firstinspires.ftc.teamcode.Robot.Systems.Claw.ClawPosition}
     */
    public void setClawPosition (Claw.ClawPosition position){
        claw.setPosition(position);
    }
    
    
    //TODO: tune the tick values to be the most optimized for our needs.
    public boolean shoulderAtPosition(){
        double shoulderTimeS = 1;
        double shoulderErrorThreshold = 3; // in degrees
        // returns true if where we are is within 20 ticks of where we want to be.
        return Math.abs(getArmAngle() - shoulderPID.getTarget()) < shoulderErrorThreshold && runtime.seconds() - shoulderTimer > shoulderTimeS;
    }
    public boolean extensionAtPosition(){
        double extensionTargetErrorThreshold = 1; // in inches
        // returns true if where we are is within 20 ticks of where we want to be.
        return Math.abs(linearSlideMotor.getCurrentPosition() - linearSlideMotor.getTargetPosition()) < extensionTargetErrorThreshold*linearSlideTicksPerInch;
    }
    public boolean elbowAtPosition(){
        double elbowTimeS = 5;
        return runtime.seconds()-elbowTimer > elbowTimeS;
    }
    
    public void saveShoulderTime(){
        shoulderTimer = runtime.seconds();
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
    
    public void switchTo(armPositions state){
        switch (state){
            case idle:
                idle();
                break;
            case dropOff:
                dropOff();
                break;
            case lowBasket:
                lowBasket();
                break;
            case highBasket:
                highBasket();
                break;
            case lowChamber:
                lowChamber();
                break;
            case highChamber:
                highChamber();
                break;
        }
    }
    
    
    
    
    
    public void highBasket(){

        //PLACEHOLDER VALUES MAYBE

        rotateTwistTo(1);
        rotateElbowTo(1);
        extendTo(1);
        rotateShoulderTo(1);
        setClawPosition(Claw.ClawPosition.open);

    }

    public void lowBasket(){

        rotateTwistTo(1);
        rotateElbowTo(1);
        extendTo(1);
        rotateShoulderTo(1);
        setClawPosition(Claw.ClawPosition.open);

    }

    public void highChamber(){

        rotateTwistTo(1);
        rotateElbowTo(1);
        extendTo(1);
        rotateShoulderTo(1);

    }

    public void lowChamber(){

        rotateTwistTo(1);
        rotateElbowTo(1);
        extendTo(1);
        rotateShoulderTo(1);

    }

    public void idle(){

        rotateTwistTo(1);
        rotateElbowTo(1);
        extendTo(1);
        rotateShoulderTo(1);

    }

    public void dropOff(){

        rotateTwistTo(1);
        rotateElbowTo(1);
        extendTo(1);
        rotateShoulderTo(1);

    }

    public void grabSpample(){

    }
    public void extensionOffset(){

    }



}