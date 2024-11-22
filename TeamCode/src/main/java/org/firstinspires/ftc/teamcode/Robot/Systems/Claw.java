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
                145,20,120);
        claw2Servo = new ServoPlus(hardwareMap.get(Servo.class,"claw2Servo"),
                145,20,120);
    }

    /**
     * Controls the four positions of the claw
     * @param position Must be one of the values from {@link org.firstinspires.ftc.teamcode.Robot.Systems.Claw.ClawPosition}
     */
    public void setPosition(ClawPosition position){
        switch (position){

            //Servo positions for the four claw states
            case open:
                claw2Servo.setServoPos(70);
                claw1Servo.setServoPos(75);
                break;
            case closed:
                claw2Servo.setServoPos(20);
                claw1Servo.setServoPos(120);
                break;
            case grabOutside:
                claw2Servo.setServoPos(22);
                claw1Servo.setServoPos(118);
                break;
            case grabInside:
                claw2Servo.setServoPos(42);
                claw1Servo.setServoPos(103);
                break;
        }
    }
}
