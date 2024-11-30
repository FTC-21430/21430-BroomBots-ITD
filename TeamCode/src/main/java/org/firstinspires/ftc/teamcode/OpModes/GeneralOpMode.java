package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.ITDbot;
import org.firstinspires.ftc.teamcode.Robot.Systems.Claw;

// Creates robot object. All OpModes inherit this.
abstract public class GeneralOpMode extends LinearOpMode {
    public ITDbot robot;
    
    public ElapsedTime runtime = new ElapsedTime();
    
    
    // variables for state machine logic
    
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
    }
    
    public boolean extensionMoved = false;
    public boolean shoulderMoved = false;
    public boolean elbowMoved = false;
    
    public armState currentArmState = armState.idle;
    
    // normal functions
    
    public void initialize() {
        robot = new ITDbot();
        robot.Init(hardwareMap,telemetry);
        
    }
    
    
    // Arm state machine logic is here because all Op-modes need to use it.
    
    public void updateState(){
        switch (currentArmState){
            case lowChamber:
                robot.spampleArm.rotateTwistTo(0);
                robot.spampleArm.rotateElbowTo(0);
                robot.spampleArm.extendTo(0);
                robot.spampleArm.rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case highBasket:
                //robot.spampleArm.setClawPosition(Claw.ClawPosition.grabOutside);
                robot.spampleArm.rotateTwistTo(90);
                robot.spampleArm.rotateElbowTo(-147);
                robot.spampleArm.extendTo(19.5);
                robot.spampleArm.rotateShoulderTo(92);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case idle:
                robot.spampleArm.rotateTwistTo(0);
                robot.spampleArm.rotateElbowTo(0);
                //robot.spampleArm.extendTo(0);
                //robot.spampleArm.rotateShoulderTo(90);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case lowBasket:
                robot.spampleArm.rotateTwistTo(90);
                robot.spampleArm.rotateElbowTo(-175);
                robot.spampleArm.extendTo(0);
                robot.spampleArm.rotateShoulderTo(100);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case dropSample:
                robot.spampleArm.rotateTwistTo(90);
                robot.spampleArm.rotateElbowTo(-45);
                robot.spampleArm.extendTo(0);
                robot.spampleArm.rotateShoulderTo(135);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case grabSample:
                robot.spampleArm.rotateTwistTo(-90);
                robot.spampleArm.rotateShoulderTo(35);
                if (robot.spampleArm.shoulderAtPosition()){
                    robot.spampleArm.rotateElbowTo(65);
                }
                break;
            case grabSample2:
                robot.spampleArm.rotateTwistTo(-90);
                robot.spampleArm.rotateShoulderTo(25);
                robot.spampleArm.rotateElbowTo(65);
                break;



//                if (!robot.spampleArm.shoulderAtPosition() || !shoulderMoved){
//                    if (!shoulderMoved) {
//                        robot.spampleArm.rotateShoulderTo(22);
//                        shoulderMoved = true;
//                    }
//                } else if (!robot.spampleArm.extensionAtPosition() || !extensionMoved) {
//                    if (!extensionMoved) {
//                        robot.spampleArm.extendTo(5);
//                        extensionMoved = true;
//                    }
//                } else {
//                    shoulderMoved = false;
//                    elbowMoved = false;
//                    extensionMoved = false;
//                }

            case highChamber:
                robot.spampleArm.rotateTwistTo(-90);
                robot.spampleArm.rotateElbowTo(5);
                robot.spampleArm.rotateShoulderTo(90);
                
                if (robot.spampleArm.shoulderAtPosition()) {
                   robot.spampleArm.extendTo(17);
                }
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case climberReady:
                robot.spampleArm.rotateTwistTo(0);
                robot.spampleArm.rotateElbowTo(1);
                robot.spampleArm.extendTo(1);
                robot.spampleArm.rotateShoulderTo(1);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case grabSpecimen:
                robot.spampleArm.rotateTwistTo(90);
                if (!robot.spampleArm.elbowAtPosition() || !elbowMoved){
                    if(!elbowMoved) {
                        robot.spampleArm.rotateElbowTo(-40);
                        elbowMoved=true;
                    }else {
                        robot.spampleArm.extendTo(5);
                        robot.spampleArm.rotateShoulderTo(118);
                        elbowMoved=false;
                        shoulderMoved=false;
                        extensionMoved=false;
                    }
                }
                
                break;
            case level1Assent:
                robot.spampleArm.rotateTwistTo(0);
                robot.spampleArm.rotateElbowTo(0);
                robot.spampleArm.extendTo(5);
                robot.spampleArm.rotateShoulderTo(70);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case specimenIdle:
                robot.spampleArm.rotateTwistTo(0);
                if (!robot.spampleArm.shoulderAtPosition() || !shoulderMoved) {
                    if (!shoulderMoved) {
                        robot.spampleArm.rotateShoulderTo(0);
                        shoulderMoved = true;
                    }
                }else {
                    robot.spampleArm.extendTo(1);
                    robot.spampleArm.rotateElbowTo(1);
                    elbowMoved=true;
                    robot.spampleArm.rotateTwistTo(1);
                    elbowMoved=false;
                    shoulderMoved=false;
                    extensionMoved=false;
                }
                
                break;
            case sampleIdle:
                robot.spampleArm.rotateTwistTo(1);
                if (!robot.spampleArm.elbowAtPosition() || !elbowMoved) {
                    if (!elbowMoved) {
                        robot.spampleArm.rotateElbowTo(1);
                        elbowMoved = true;
                    }
                } else if (!robot.spampleArm.extensionAtPosition() || !extensionMoved) {
                    if (!extensionMoved) {
                        robot.spampleArm.extendTo(1);
                        extensionMoved = true;
                    }
                } else if (!robot.spampleArm.shoulderAtPosition() || !shoulderMoved) {
                    if (!shoulderMoved) {
                        robot.spampleArm.rotateShoulderTo(1);
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
               // robot.spampleArm.rotateTwistTo(-90);
                //robot.spampleArm.rotateElbowTo(10);
                robot.spampleArm.extendTo(13);
                //robot.spampleArm.rotateShoulderTo(86);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
                break;
            case intake:
                robot.spampleArm.rotateShoulderTo(30);
                break;
            case spearHead:
                robot.spampleArm.rotateTwistTo(0);
                robot.spampleArm.rotateElbowTo(0);
                robot.spampleArm.extendTo(0);
                robot.spampleArm.rotateShoulderTo(10);
                shoulderMoved = false;
                elbowMoved = false;
                extensionMoved = false;
        }
    }
    
}
