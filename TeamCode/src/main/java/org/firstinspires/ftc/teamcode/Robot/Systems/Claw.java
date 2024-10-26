package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    //The servos which the claw uses
    ServoPlus claw1Servo;
    ServoPlus claw2Servo;

    /**
     * The four positions/states of the claw
     * open
     * closed
     * grabOutside
     * grabInside
     */
    public enum ClawPosition{
        open,
        closed,
        grabOutside,
        grabInside,
    }

    /**
     * Claw constructor
     * @param hardwareMap Robot hardware map
     */
    public Claw (HardwareMap hardwareMap){
        //Mapping/initializing the claw
        claw1Servo = new ServoPlus(hardwareMap.get(Servo.class,"claw1Servo"),
                180,0,180);
        claw2Servo = new ServoPlus(hardwareMap.get(Servo.class,"claw2Servo"),
                180,0,180);
    }

    /**
     * Controls the four positions of the claw
     * @param position Must be one of the values from {@link org.firstinspires.ftc.teamcode.Robot.Systems.Claw.ClawPosition}
     */
    public void setPosition(ClawPosition position){
        switch (position){
            //TODO: un-zero these values
            //Servo positions for the four claw states
            case open:
                claw1Servo.setServoPos(50);
                claw2Servo.setServoPos(130);
                break;
            case closed:
                claw1Servo.setServoPos(0);
                claw2Servo.setServoPos(0);
                break;
            case grabOutside:
                claw1Servo.setServoPos(30);
                claw2Servo.setServoPos(150);
                break;
            case grabInside:
                claw1Servo.setServoPos(40);
                claw2Servo.setServoPos(140);
                break;
        }
    }
}
