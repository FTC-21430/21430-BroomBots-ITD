package org.firstinspires.ftc.teamcode.Robot.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    ServoPlus claw1Servo;
    ServoPlus claw2Servo;

    public enum ClawPosition{
        open,
        closed,
        grabOutside,
        grabInside,
    }

    public Claw (HardwareMap hardwareMap){
        claw1Servo = new ServoPlus(hardwareMap.get(Servo.class,"claw1Servo"),
                180,0,180);
        claw2Servo = new ServoPlus(hardwareMap.get(Servo.class,"claw2Servo"),
                180,0,180);
    }
    public void setPosition(ClawPosition position){
        switch (position){

            //TODO: un-zero these values
            case open:
                claw1Servo.setServoPos(0);
                claw2Servo.setServoPos(0);
                break;
            case closed:
                claw1Servo.setServoPos(0);
                claw2Servo.setServoPos(0);
                break;
            case grabOutside:
                claw1Servo.setServoPos(0);
                claw2Servo.setServoPos(0);
                break;
            case grabInside:
                claw1Servo.setServoPos(0);
                claw2Servo.setServoPos(0);
                break;
        }
    }
}
