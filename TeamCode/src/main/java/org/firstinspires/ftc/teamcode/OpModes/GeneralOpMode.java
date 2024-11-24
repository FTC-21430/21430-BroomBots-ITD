package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.ITDbot;
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
    climberReady,
    level1Assent,
    dropSample,
    specimenIdle,
    sampleIdle,
    scoreHighChamber,
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
          robot.spampleArm.rotateTwistTo(1);
          robot.spampleArm.rotateElbowTo(1);
          robot.spampleArm.extendTo(1);
          robot.spampleArm.rotateShoulderTo(1);
          shoulderMoved = false;
          elbowMoved = false;
          extensionMoved = false;
          break;
        case highBasket:
          robot.spampleArm.rotateTwistTo(180);
          robot.spampleArm.rotateElbowTo(-180);
          robot.spampleArm.extendTo(19.5);
          robot.spampleArm.rotateShoulderTo(110);
          shoulderMoved = false;
          elbowMoved = false;
          extensionMoved = false;
          break;
        case idle:
          robot.spampleArm.rotateTwistTo(90);
          robot.spampleArm.rotateElbowTo(0);
          robot.spampleArm.extendTo(0);
          robot.spampleArm.rotateShoulderTo(90);
          shoulderMoved = false;
          elbowMoved = false;
          extensionMoved = false;
          break;
        case lowBasket:
          robot.spampleArm.rotateTwistTo(1);
          robot.spampleArm.rotateElbowTo(1);
          robot.spampleArm.extendTo(1);
          robot.spampleArm.rotateShoulderTo(1);
          shoulderMoved = false;
          elbowMoved = false;
          extensionMoved = false;
          break;
        case dropSample:
          robot.spampleArm.rotateTwistTo(1);
          robot.spampleArm.rotateElbowTo(1);
          robot.spampleArm.extendTo(1);
          robot.spampleArm.rotateShoulderTo(1);
          shoulderMoved = false;
          elbowMoved = false;
          extensionMoved = false;
          break;
        case grabSample:
          robot.spampleArm.rotateTwistTo(0);
          robot.spampleArm.rotateElbowTo(65);
          if (!robot.spampleArm.shoulderAtPosition() || !shoulderMoved){
            if (!shoulderMoved) {
              robot.spampleArm.rotateShoulderTo(26);
              shoulderMoved = true;
            }
          } else if (!robot.spampleArm.extensionAtPosition() || !extensionMoved) {
            if (!extensionMoved) {
              robot.spampleArm.extendTo(5);
              extensionMoved = true;
            }
          } else {
            shoulderMoved = false;
            elbowMoved = false;
            extensionMoved = false;
          }
          break;
        case highChamber:
          robot.spampleArm.rotateTwistTo(0);
          robot.spampleArm.rotateElbowTo(180);
          robot.spampleArm.extendTo(0);
          robot.spampleArm.rotateShoulderTo(40);
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
          robot.spampleArm.rotateTwistTo(180);
          if (!robot.spampleArm.elbowAtPosition() || !elbowMoved){
            if(!elbowMoved) {
              robot.spampleArm.rotateElbowTo(-76);
              elbowMoved=true;
            }else {
              robot.spampleArm.extendTo(0);
              robot.spampleArm.rotateShoulderTo(148);
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
          robot.spampleArm.rotateTwistTo(0);
          robot.spampleArm.rotateElbowTo(150);
          robot.spampleArm.extendTo(0);
          robot.spampleArm.rotateShoulderTo(40);
          shoulderMoved = false;
          elbowMoved = false;
          extensionMoved = false;
          break;
      }
    }
    
}
