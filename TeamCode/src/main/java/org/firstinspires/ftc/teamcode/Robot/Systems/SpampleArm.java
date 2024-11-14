package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//This class is the code foundations for making the robot's arm move.
public class SpampleArm {
    //Actuators for the arm
    DcMotor shoulderMotor;
    DcMotor linearSlideMotor;
    ServoPlus wristServo;
    ServoPlus twistServo;
    Claw claw;

    double targetExtension = 0;
    
    //TODO: replace with correct value; calibrated for 312 RPM motor
    //the motor will not turn correctly without these values right.
    //Constants for the shoulder
    final double shoulderPulsesPerRevolution = 537.7;
    final double shoulderTicksPerDegrees = shoulderPulsesPerRevolution / 360;
    
    // used to correct the error caused in the slide by the rotation of the shoulder.
    // TODO: tune this value
    final double shoulderRotationToSlide = 0.02;

    //Constants for the linear slide
    final double linearSlidePulsesPerRevolution = 537.7;
    final double linearSlideRevPerInch = 1;
    final double linearSlideTicksPerInch = linearSlidePulsesPerRevolution * linearSlideRevPerInch;
    final double linearSlideMaxExtension = 23;

    /**
     * Arm constructor
     * @param hardwareMap Robot hardware map
     */
    public SpampleArm (HardwareMap hardwareMap){
        //TODO: confirm names
        //Mapping/initializing motors
        shoulderMotor = hardwareMap.get(DcMotor.class,"shoulderMotor");
        shoulderMotor.setTargetPosition(0);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // you need to set how fast the motor moves before it will move at all.
        shoulderMotor.setPower(1);

        linearSlideMotor = hardwareMap.get(DcMotor.class,"linearSlideMotor");
        linearSlideMotor.setTargetPosition(0);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // you need to set how fast the motor moves before it will move at all.
        linearSlideMotor.setPower(1);

        //Mapping/initializing servos
        wristServo = new ServoPlus(hardwareMap.get(Servo.class,"wristServo"),
                180,-0.00001,180);
        //wristServo makes wrist go up and down

        twistServo = new ServoPlus(hardwareMap.get(Servo.class,"twistServo"),
                180,0,180);
        //twistServo makes wrist twist

        claw = new Claw(hardwareMap);
    }

    /**
     * Controls the shoulder motor
     * @param angle Angle for shoulder in degrees
     */
    public void rotateShoulderTo (double angle){
        //this ensures that the rotation of the robot's arm is never past the mechanical constraints of the robot
        if (angle < 0) {
            angle = 0;
        }
        if (angle > 180) {
            angle = 180;
        }
        shoulderMotor.setTargetPosition((int) (angle * shoulderTicksPerDegrees));
    }
    
    /**
     * because the shoulder could be moving between setter calls of the linear slide, we have to update is constantly to correct.
     */
    public void updateSlide(){
        linearSlideMotor.setTargetPosition((int) ((targetExtension * linearSlideTicksPerInch) - (shoulderMotor.getCurrentPosition() * shoulderRotationToSlide)));
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
     * Controls the wrist motor
     * @param angle Angle for wrist in degrees
     */
    public void rotateWristTo (double angle){
        wristServo.setServoPos(angle);
    }

    /**
     * Controls the twist of the wrist
     * @param angle Angle for twist in degrees
     */
    public void rotateTwistTo (double angle){
        twistServo.setServoPos(angle);
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
    Wrist Twist (don't) -Joystick right
  X High Basket         -Y
  X Low Basket          -X
  X High Chamber        -B
  X Low Chamber         -A

     */
    //shoulder
    //extensor
    //elbow :sob: ADD THE ELBOW!!?!!
    //twist
    //wrist
    //pinchy

    //High Basket
    //fix variables
    public void highBasket(){

        //PLACEHOLDER VALUES MAYBE

        rotateTwistTo(1);
        rotateWristTo(1);
        extendTo(1);
        rotateShoulderTo(1);
        setClawPosition(Claw.ClawPosition.open);

    }

    public void lowBasket(){

        rotateTwistTo(1);
        rotateWristTo(1);
        extendTo(1);
        rotateShoulderTo(1);
        setClawPosition(Claw.ClawPosition.open);

    }

    public void highChamber(){

        setClawPosition(Claw.ClawPosition.closed);
        rotateTwistTo(1);
        rotateWristTo(1);
        extendTo(1);
        rotateShoulderTo(1);

    }

    public void lowChamber(){

        setClawPosition(Claw.ClawPosition.closed);
        rotateTwistTo(1);
        rotateWristTo(1);
        extendTo(1);
        rotateShoulderTo(1);

    }

    public void idle(){

        setClawPosition(Claw.ClawPosition.closed);
        rotateTwistTo(1);
        rotateWristTo(1);
        extendTo(1);
        rotateShoulderTo(1);

    }

    public void dropOff(){

        setClawPosition(Claw.ClawPosition.open);
        rotateTwistTo(1);
        rotateWristTo(1);
        extendTo(1);
        rotateShoulderTo(1);

    }

    public void grabSpample(){

    }
    public void extensionOffset(){

    }



}