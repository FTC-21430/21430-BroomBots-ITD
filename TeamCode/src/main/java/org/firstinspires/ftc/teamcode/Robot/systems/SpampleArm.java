package org.firstinspires.ftc.teamcode.Robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SpampleArm {
    DcMotor shoulderMotor;
    DcMotor linearSlideMotor;
    ServoPlus wristServo;
    ServoPlus twistServo;
    Claw claw;

    //PPR is Pulses Per Revolution
    //TODO: replace with correct value; calibrated for 312 RPM motor
    double shoulderPPR = 537.7;
    double shoulderTicksPerDegrees = shoulderPPR / 360;

    double linearSlidePPR = 537.7;
    double linearSlideRevPerInch = 1;
    double linearSlideTicksPerInch = linearSlidePPR * linearSlideRevPerInch;
    double linearSlideMaxExtension = 23;

    public SpampleArm (HardwareMap hardwareMap){
        //TODO: confirm names
        shoulderMotor = hardwareMap.get(DcMotor.class,"shoulderMotor");
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearSlideMotor = hardwareMap.get(DcMotor.class,"linearSlideMotor");
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wristServo = new ServoPlus(hardwareMap.get(Servo.class,"wristServo"),
                180,0,180);

        twistServo = new ServoPlus(hardwareMap.get(Servo.class,"twistServo"),
                180,0,180);

        claw = new Claw(hardwareMap);
    }

    public void rotateShoulderTo (double angle){
        if (angle < 0) {
            angle = 0;
        }
        if (angle < 180) {
            angle = 180;
        }
        shoulderMotor.setTargetPosition((int) (angle * shoulderTicksPerDegrees));
    }

    public void extendTo (double inches){
        if (inches < 0){
            inches = 0;
        }
        if (inches > linearSlideMaxExtension) {
            inches = linearSlideMaxExtension;
        }
        linearSlideMotor.setTargetPosition((int) (inches * linearSlideTicksPerInch));
    }

    public void rotateWristTo (double angle){
        wristServo.setServoPos(angle);
    }
    public void rotateTwistTo (double angle){
        twistServo.setServoPos(angle);
    }
    public void setClawPosition (Claw.ClawPosition position){
        claw.setPosition(position);
    }

}
