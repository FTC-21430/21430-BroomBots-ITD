package org.firstinspires.ftc.teamcode.Robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SpampleArm {
    //Actuators for the arm
    DcMotor shoulderMotor;
    DcMotor linearSlideMotor;
    ServoPlus wristServo;
    ServoPlus twistServo;
    Claw claw;

    //TODO: replace with correct value; calibrated for 312 RPM motor
    //Constants for the shoulder
    double shoulderPulsesPerRevolution = 537.7;
    double shoulderTicksPerDegrees = shoulderPulsesPerRevolution / 360;

    //Constants for the linear slide
    double linearSlidePulsesPerRevolution = 537.7;
    double linearSlideRevPerInch = 1;
    double linearSlideTicksPerInch = linearSlidePulsesPerRevolution * linearSlideRevPerInch;
    double linearSlideMaxExtension = 23;

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

        linearSlideMotor = hardwareMap.get(DcMotor.class,"linearSlideMotor");
        linearSlideMotor.setTargetPosition(0);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Mapping/initializing servos
        wristServo = new ServoPlus(hardwareMap.get(Servo.class,"wristServo"),
                180,-0.00001,180);

        twistServo = new ServoPlus(hardwareMap.get(Servo.class,"twistServo"),
                180,0,180);

        claw = new Claw(hardwareMap);
    }

    /**
     * Controls the shoulder motor
     * @param angle Angle for shoulder in degrees
     */
    public void rotateShoulderTo (double angle){
        if (angle < 0) {
            angle = 0;
        }
        if (angle < 180) {
            angle = 180;
        }
        shoulderMotor.setTargetPosition((int) (angle * shoulderTicksPerDegrees));
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
        linearSlideMotor.setTargetPosition((int) (inches * linearSlideTicksPerInch));
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
     * @param angle Angle for twist rotation in degrees
     */
    public void rotateTwistTo (double angle){
        twistServo.setServoPos(angle);
    }

    /**
     * Controls the four positions of the claw
     * @param position Must be one of the values from {@link org.firstinspires.ftc.teamcode.Robot.systems.Claw.ClawPosition}
     */
    public void setClawPosition (Claw.ClawPosition position){
        claw.setPosition(position);
    }

}