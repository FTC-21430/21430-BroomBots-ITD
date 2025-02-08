package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Resources.PIDController;


//This class is the code foundations for making the robot's arm move.
@Config
public class SpampleArm {

    
    private ElapsedTime runtime = null;
    private double elbowAngleOffset = 161.3;

    private double shoulderAngleOffset;

    public boolean extensionMoved = false;
    public boolean shoulderMoved = false;
    public boolean elbowMoved = false;

    public static double spicemenGrabElbowTeleop =-40;
    public static double spicemenGrabExtensionTeleop = 4.7;
    public static double spicemenGrabShoulderTeleop = 119;
    public static double specimenGrabTwistTeleop = 83;

    public static double spicemenGrabElbowAuto =-46.5;
    public static double spicemenGrabExtensionAuto = 5.0;
    public static double spicemenGrabShoulderAuto = 119.3;
    public static double specimenGrabTwistAuto = 83;

    public armState currentArmState = armState.idle;

    //Arm sensors
    public AnalogInput armPotentiometer = null;

    private DigitalChannel armLimitSwitch = null;
    
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
    // goBILDA 60rpm motor spec sheet 2786.2 pulses per revolution at output, sprockets are 3:1 ratio
    // 2786.2 * 3 = 8358.6
    final double shoulderPulsesPerRevolution = 8358.6;
    final double shoulderTicksPerDegrees = shoulderPulsesPerRevolution / 360;
    
    //Constants for the linear slide
    //REV Gear cartridges 5, 3, 3 = 5.23 * 2.89 * 2.89 = 43.681283 : 1 ratio
    //Encoder 28 pulse per revolution = 1223.081
    final double linearSlidePulsesPerRevolution = 1223.081;
    // GT2 pulley is 60 teeth, 2mm pitch = 120mm inch per revolution
    // 120mm / 2.54 = 4.724409 inch per revolution
    // multiplied by two because of the cascade rigging
    final double linearSlideRevPerInch = 1/(4.724409*2);
    final double linearSlideTicksPerInch = linearSlidePulsesPerRevolution * linearSlideRevPerInch;
    final double linearSlideMaxExtension = 19.5;
    
    // used to correct the error caused in the slide by the rotation of the shoulder.
    final double shoulderRotationToSlide = -linearSlidePulsesPerRevolution/shoulderPulsesPerRevolution;


//    shoulder constants
//    should be able to be changed by ftc dashboard
    public static double pConstant = 0.028;
    public static double iConstant = 0.06;
    public static double dConstant =0.0005;

//    the shoulder constants for when the arm is up high without the integrator

    private double pConstantHigh = 0.035;
    private double dConstantHigh = 0.0023;

    private double dConstantGrab = 0.0025;

    private boolean startedCalibratingExtension = false;
    private double calibrationTimer = 0;

    private boolean calibReleasing = false;
    private boolean calibRetracting = false;

    private double shoulderTimer = 0.0;
    private Telemetry telemetry = null;
    
    /**
     * Arm constructor
     * @param hardwareMap Robot hardware map
     */
    public SpampleArm (HardwareMap hardwareMap, ElapsedTime runtime, boolean reset, boolean isAuto, Telemetry telemetry){

        
        shoulderPID = new PIDController(pConstant, iConstant,dConstant, new ElapsedTime());
        shoulderPID.setTarget(90);

        //Mapping/initializing motors
        shoulderMotor = hardwareMap.get(DcMotor.class,"shoulderMotor");
        
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // you need to set how fast the motor moves before it will move at all.


        linearSlideMotor = hardwareMap.get(DcMotor.class,"linearSlideMotor");
        linearSlideMotor.setTargetPosition(0);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // you need to set how fast the motor moves before it will move at all.
        linearSlideMotor.setPower(1);

        armLimitSwitch = hardwareMap.get(DigitalChannel.class, "extensionLimitSwitch");
        armLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        //Mapping/initializing servos

        if (isAuto){
            elbowServo = new ServoPlus(hardwareMap.get(Servo.class,"elbowServo"),
                    250,0,290 );
        }else {
            elbowServo = new ServoPlus(hardwareMap.get(Servo.class, "elbowServo"),
                    264.3, 0, 264.3);
        }
        twistServo = new ServoPlus(hardwareMap.get(Servo.class,"twistServo"),
                260,0,260);

        claw = new Claw(hardwareMap);
        
        armPotentiometer = hardwareMap.get(AnalogInput.class, "shoulderAngleP");
        shoulderAngleOffset = getArmAngle();
        
        this.runtime = runtime;
        this.telemetry = telemetry;

    }

    
    
    public double getArmAngle(){
//        return 38.412 * Math.pow(armPotentiometer.getVoltage(),2) - 232.78 * armPotentiometer.getVoltage() + 299.5 - robotTilt;
//       return  armPotentiometer.getVoltage();

//        return -139.94 * armPotentiometer.getVoltage() + 243.28;

//         current tuned potentiometer as of 1/11/....25

        return -31.767 * Math.pow(armPotentiometer.getVoltage(),3) + 154.13 * Math.pow(armPotentiometer.getVoltage(),2) - 367.2 * armPotentiometer.getVoltage() + 345.4;


//        return 0.0018 * Math.pow(armPotentiometer.getVoltage(), 2) + 4.9578 * armPotentiometer.getVoltage() + 5.1362 - robotTilt;
    }

    public double getArmExtension(){
        return linearSlideMotor.getCurrentPosition() / linearSlideTicksPerInch;
    }

    public void calibrateExtension(){
        startedCalibratingExtension = true;
        currentArmState = armState.calibrating;
        calibrationTimer = runtime.seconds();
    }

    /**
     * Controls the shoulder motor
     * @param angle Angle for shoulder in degrees
     */

    public void rotateShoulderTo (double angle){
        
        // I tried to do some fancy calibration and stuff but it did not work :(
//        double correctedAngle = angle - (6.33 + 9.66E-03 * angle + -1.12E-03 * Math.pow(angle, 2));
//        double correctedAngle;
//        if (angle <= 30){
//            // to acount for the error of 7.2 degrees when picking up from angles less than 30 degrees
//            correctedAngle = angle + 7.2;
//        }
//
//
//        else{
//            correctedAngle = angle;
//        }
        
        
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
    public void updateArm(boolean isAuto){
//        int armSafetyOffset = 0;
        // broke the robot last time we checked :( 2/5/25 - Tobin
//        if (currentArmState != armState.init && currentArmState != armState.test){
//            // ensures the arm is within 5 degrees to vertical.
//            if (armLimitSwitch.getState() && getArmAngle() > 90 - 5 && getArmAngle() < 90 + 5){
//                linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }else if(armLimitSwitch.getState()){
//                armSafetyOffset = 100;
//            }
//        }


        if (getArmAngle() < 0 || getArmAngle() > 180){
            RobotLog.e("ARM POTENTIOMETER IS LIKELY BROKEN: CHECK WIRING, WILL NOT UPDATE ARM");
            telemetry.addLine("ARM POTENTIOMETER IS LIKELY BROKEN: CHECK WIRING, WILL NOT UPDATE ARM");

        }else {
            if (isAuto) {
                updateStateAutonomous();
            } else {
                updateTeleopState();
            }
            if (getArmExtension() >= 7) {
                shoulderPID.updateConstants(pConstantHigh, iConstant, dConstantHigh);
                shoulderPID.setIntegralMode(false);
            } else if (currentArmState == armState.grabSample2) {
                shoulderPID.updateConstants(pConstant, iConstant, dConstantGrab);
            } else {
                shoulderPID.updateConstants(pConstant, iConstant, dConstant);
                shoulderPID.setIntegralMode(true);
            }

//        shoulderPID.updateConstants(pConstant, iConstant, dConstant);
            shoulderPID.update(getArmAngle());
            shoulderMotor.setPower(shoulderPID.getPower() + Math.cos(getArmAngle() * Math.PI / 180) * 0.12);
            linearSlideMotor.setTargetPosition((int) ((targetExtension * linearSlideTicksPerInch) + (shoulderMotor.getCurrentPosition() * shoulderRotationToSlide)));
        }
    }


    public boolean getArmLimitSwitchPressed(){
        return armLimitSwitch.getState();
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
        twistServo.setServoPos(angle+11+ 90);
    }

    public double getTwist(){
        return twistServo.getServoPos() - 11 - 90;

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


    public void updateShoulderConstants(){
        shoulderPID.updateConstants(pConstant,iConstant,dConstant);
    }

    ;
    public double getElbowRotation(){
        return elbowServo.getServoPos() - elbowAngleOffset;
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
        test,
        fullyIdle,
        pictureTake,
        calibrating

    }

    public void updateStateAutonomous(){

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
                rotateElbowTo(-125);
                extendTo(19);
                rotateShoulderTo(94);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case idle:
                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(2);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case fullyIdle:
                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(0.0);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case lowBasket:
                rotateTwistTo(90);
                rotateElbowTo(-142);
                extendTo(2);
                rotateShoulderTo(100);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case dropSample:

                rotateTwistTo(90);
                rotateElbowTo(-45);
                extendTo(2);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case grabSample:
                rotateShoulderTo(30);
                extendTo(2);
                rotateElbowTo(65.5);
                break;
            case grabSample2:
                extendTo(2);
                rotateShoulderTo(16);
                rotateElbowTo(67);
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
                rotateShoulderTo(90);
                extendTo(1);

                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case climberReady:

                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(1);
                rotateShoulderTo(130);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case grabSpecimen:

                rotateTwistTo(specimenGrabTwistAuto);
                if (!elbowAtPosition() || !elbowMoved){
                    if(!elbowMoved) {
                        rotateElbowTo(spicemenGrabElbowAuto);
                        elbowMoved=true;
                    }else {
                        extendTo(spicemenGrabExtensionAuto);
                        rotateShoulderTo(spicemenGrabShoulderAuto);
                        elbowMoved=false;
                        shoulderMoved=false;
                        extensionMoved=false;
                    }
                }

                break;
            case level1Assent:

                rotateTwistTo(0);
                rotateElbowTo(60);
                extendTo(0);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case specimenIdle:

                rotateTwistTo(0);
                if (!shoulderAtPosition() || !shoulderMoved) {
                    if (!shoulderMoved) {
                        rotateShoulderTo(90);
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
                        rotateShoulderTo(90);
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
                rotateShoulderTo(90);
                break;
            case spearHead:

                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(0);
                rotateShoulderTo(90);
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
            case pictureTake:
                extendTo(2);
                rotateTwistTo(0);
                rotateElbowTo(20);
                rotateShoulderTo(30);
            case test:
                break;
            case calibrating:
                rotateShoulderTo(90);
                if (startedCalibratingExtension){
                    startedCalibratingExtension = false;
                    if (getArmLimitSwitchPressed()){
                        calibReleasing = true;
                        linearSlideMotor.setPower(0);
                    }else {
                        calibRetracting = true;
                        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        linearSlideMotor.setPower(-0.55);
                    }
                } else{
                    if (calibReleasing){
                        if (runtime.seconds() > calibrationTimer + 0.4){
                            linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            linearSlideMotor.setPower(1);
                            extendTo(2);
                            currentArmState = armState.idle;
                        }
                    } else if(calibRetracting){
                        if (getArmLimitSwitchPressed()){
                            linearSlideMotor.setPower(0);
                            linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            linearSlideMotor.setPower(1);
                            extendTo(2);
                            currentArmState = armState.idle;
                        }
                    }else{
                        currentArmState = armState.idle;
                    }
                }

                break;

        }
    }
    public void updateTeleopState(){

        switch (currentArmState){
            case lowChamber:

                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(2);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case highBasket:

                //setClawPosition(Claw.ClawPosition.grabOutside);
                rotateTwistTo(90);
                rotateElbowTo(-125);
                extendTo(19);
                rotateShoulderTo(94);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case idle:
                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(2);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case fullyIdle:
                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(0.0);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case lowBasket:
                rotateTwistTo(90);
                rotateElbowTo(-142);
                extendTo(2);
                rotateShoulderTo(100);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case dropSample:

                rotateTwistTo(90);
                rotateElbowTo(-45);
                extendTo(2);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case grabSample:
                rotateShoulderTo(30);
                extendTo(2);
                rotateElbowTo(65.5);
                break;
            case grabSample2:
                extendTo(2);
                rotateShoulderTo(16.5);
                rotateElbowTo(73.5);
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
                rotateShoulderTo(90);
                extendTo(2.25);

                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case climberReady:

                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(1);
                rotateShoulderTo(130);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case grabSpecimen:

                rotateTwistTo(specimenGrabTwistTeleop);
                if (!elbowAtPosition() || !elbowMoved){
                    if(!elbowMoved) {
                        rotateElbowTo(spicemenGrabElbowTeleop);
                        elbowMoved=true;
                    }else {
                        extendTo(spicemenGrabExtensionTeleop);
                        rotateShoulderTo(spicemenGrabShoulderTeleop);
                        elbowMoved=false;
                        shoulderMoved=false;
                        extensionMoved=false;
                    }
                }

                break;
            case level1Assent:

                rotateTwistTo(0);
                rotateElbowTo(60);
                extendTo(0);
                rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case specimenIdle:

                rotateTwistTo(0);
                if (!shoulderAtPosition() || !shoulderMoved) {
                    if (!shoulderMoved) {
                        rotateShoulderTo(90);
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
                        rotateShoulderTo(90);
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
                rotateShoulderTo(90);
                break;
            case spearHead:

                rotateTwistTo(0);
                rotateElbowTo(0);
                extendTo(0);
                rotateShoulderTo(90);
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
            case pictureTake:
                extendTo(2);
                rotateTwistTo(0);
                rotateElbowTo(20);
                rotateShoulderTo(30);
            case test:
                break;
            case calibrating:
                if (startedCalibratingExtension){
                    startedCalibratingExtension = false;
                    if (getArmLimitSwitchPressed()){
                        calibReleasing = true;
                        linearSlideMotor.setPower(0);
                    }else {
                        calibRetracting = true;
                        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        linearSlideMotor.setPower(-0.55);
                    }
                    } else{
                        if (calibReleasing){
                         if (runtime.seconds() > calibrationTimer + 0.4){
                            linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            linearSlideMotor.setPower(1);
                            extendTo(2);
                            currentArmState = armState.idle;
                         }
                        } else if(calibRetracting){
                        if (getArmLimitSwitchPressed()){
                            linearSlideMotor.setPower(0);
                            linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            linearSlideMotor.setPower(1);
                            extendTo(2);
                            currentArmState = armState.idle;
                        }
                    }else{
                        currentArmState = armState.idle;
                    }
                }

                break;
        }
    }
}
