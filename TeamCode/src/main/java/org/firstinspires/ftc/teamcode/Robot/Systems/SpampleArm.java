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
    private double elbowAngleOffset = 176;
    private double shoulderAngleOffset;

    public boolean extensionMoved = false;
    public boolean shoulderMoved = false;
    public boolean elbowMoved = false;

    public armState currentArmState = armState.idle;

    //Arm sensors
    public AnalogInput armPotentiometer = null;
    
    //Actuators for the arm

    
    public PIDController shoulderPID;

    public DcMotor shoulderMotor;
    DcMotor linearSlideMotor;
    ServoPlus elbowServo;
    
    double elbowTimer =0.0;
    
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
    final double linearSlideRevPerInch = 1/(4.724*2);
    final double linearSlideTicksPerInch = linearSlidePulsesPerRevolution * linearSlideRevPerInch;
    final double linearSlideMaxExtension = 19.5;
    
    // used to correct the error caused in the slide by the rotation of the shoulder.
    final double shoulderRotationToSlide = -linearSlidePulsesPerRevolution/shoulderPulsesPerRevolution;

    public static double pConstant = 0.028;
    public static double iConstant = 0.06;
    public static double dConstant =0.0005;




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
                250,0,290 );

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
        if (angle > 178) {
            angle = 178;
        }

        shoulderPID.setTarget(angle);


    }
    
    /**
     * because the shoulder could be moving between setter calls of the linear slide, we have to update is constantly to correct.
     */
    public void updateArm(){
        updateState();
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

//        angle = angle / ELBOWCONSTANT;

        elbowServo.setServoPos(angle+elbowAngleOffset);
        elbowTimer = runtime.milliseconds();
    }

    /**
     * Controls the twist of the twist
     * @param angle Angle for twist in degrees
     */
    public void rotateTwistTo (double angle){
        // values to ensure the twist goes where we need it to, then rotated by 90 degrees
        twistServo.setServoPos(angle+10+ 90);
    }

    public double getTwist(){
        return twistServo.getServoPos() - 16 - 90;

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
        double shoulderTimeS = 1.5;
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

    public enum armState {
        highBasket,
        lowBasket,
        highChamber,
        lowChamber,
        idle,
        grabSpecimen,
        grabSample,
        grabSample2,
        climberReady,
        level1Assent,
        dropSample,
        specimenIdle,
        sampleIdle,
        scoreHighChamber,
        spearHead,
        intake,
        init,
        test
    }

    public void updateState(){

        switch (currentArmState){
            case lowChamber:

                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(0);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case highBasket:

                //setClawPosition(Claw.ClawPosition.grabOutside);
                rotateTwistTo(90);
                rotateElbowTo(-130);
                extendTo(19.5);
                rotateShoulderTo(94);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case idle:
                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(0);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case lowBasket:

                rotateTwistTo(90);
                rotateElbowTo(-130);
                extendTo(0);
                rotateShoulderTo(100);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case dropSample:

                rotateTwistTo(90);
                rotateElbowTo(-45);
                extendTo(0);
                rotateShoulderTo(135);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case grabSample:

                rotateShoulderTo(35);

                if (shoulderAtPosition()){
                    rotateElbowTo(65);
                }
                break;
            case grabSample2:

                rotateShoulderTo(22);
                rotateElbowTo(65);
                break;



//                if (!shoulderAtPosition() || !shoulderMoved){
//                    if (!shoulderMoved) {
//                        rotateShoulderTo(22);
//                        shoulderMoved = true;
//                    }
//                } else if (!extensionAtPosition() || !extensionMoved) {
//                    if (!extensionMoved) {
//                        extendTo(5);
//                        extensionMoved = true;
//                    }
//                } else {
//                    shoulderMoved = false;
//                    elbowMoved = false;
//                    extensionMoved = false;
//                }

            case highChamber:

                rotateTwistTo(-90);
                rotateElbowTo(90);
                rotateShoulderTo(90);

                if (shoulderAtPosition()) {
                    extendTo(1.5);
                }
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case climberReady:

                rotateTwistTo(0);
                rotateElbowTo(1);
                extendTo(1);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case grabSpecimen:

                rotateTwistTo(90);
                if (!elbowAtPosition() || !elbowMoved){
                    if(!elbowMoved) {
                        rotateElbowTo(-30);
                        elbowMoved=true;
                    }else {
                        extendTo(5.5);
                        rotateShoulderTo(120.5);
                        elbowMoved=false;
                        shoulderMoved=false;
                        extensionMoved=false;
                    }
                }

                break;
            case level1Assent:

                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(0);
                rotateShoulderTo(82);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case specimenIdle:

                rotateTwistTo(0);
                if (!shoulderAtPosition() || !shoulderMoved) {
                    if (!shoulderMoved) {
                        rotateShoulderTo(0);
                        shoulderMoved = true;
                    }
                }else {
                    extendTo(1);
                    rotateElbowTo(1);
                    elbowMoved=true;
                    rotateTwistTo(1);
                    elbowMoved=false;
                    shoulderMoved=false;
                    extensionMoved=false;
                }

                break;
            case sampleIdle:

                rotateTwistTo(1);
                if (!elbowAtPosition() || !elbowMoved) {
                    if (!elbowMoved) {
                        rotateElbowTo(1);
                        elbowMoved = true;
                    }
                } else if (!extensionAtPosition() || !extensionMoved) {
                    if (!extensionMoved) {
                        extendTo(1);
                        extensionMoved = true;
                    }
                } else if (!shoulderAtPosition() || !shoulderMoved) {
                    if (!shoulderMoved) {
                        rotateShoulderTo(1);
                        shoulderMoved = true;
                    }
                }
                else{
                    elbowMoved=false;
                    shoulderMoved=false;
                    extensionMoved=false;
                }
                break;
            case scoreHighChamber:

                // rotateTwistTo(-90);
                //rotateElbowTo(10);
                extendTo(14.5);
                //rotateShoulderTo(86);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case intake:
                rotateShoulderTo(30);
                break;
            case spearHead:

                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(0);
                rotateShoulderTo(10);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
            case init:
                setClawPosition(Claw.ClawPosition.closed);
                extendTo(0);
                rotateTwistTo(0);
                rotateElbowTo(0);
                rotateShoulderTo(137.5);
                break;
            case test:
                break;
        }


    }
    
    
    
    
    



}