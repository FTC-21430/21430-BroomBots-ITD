package org.firstinspires.ftc.teamcode.resourses;

public class InverseKinematics {

    private double armLength;
    private double armRotation;
    private double wristRotation;
    private double wristTwist;
    private double robotX, robotY;

    private double robotAngle;


    //Todo tune these values correctly
    private final double armExtensionMax = 1.0, armExtensionMin = 1.0;

    private final double armRotationMin = 1.0, armRotationMax = 1.0;

    private final double wristRotMin = 1.0, wristRotMax = 1.0;

    private final double wristTwistMin = 1.0, wristTwistMax = 1.0;



    private enum sides {
        redFront,
        redSide,
        blueFront,
        blueSide
    }

    private sides currentSide;


    //TODO these values are from CAD, tune these more accurate to the real robot
    private final double wristLength = 8.95;

    private final double pivotOffset = 5.55;

    public InverseKinematics() {

    }

    public void calculateKinematics(double xCurrent, double yCurrent, double targetX, double targetY, double targetZ, double targetAngle) {
        robotX = xCurrent;
        robotY = yCurrent;
        if (xCurrent <= -25){
            currentSide = sides.blueFront;
            blueFront(targetX, targetY, targetZ, targetAngle);
        } else if (xCurrent >= 25) {
            currentSide = sides.redFront;
            redFront(targetX,targetY,targetZ, targetAngle);
        } else if (yCurrent >= 0) {
            currentSide = sides.redSide;
            redSide(targetX,targetY,targetZ, targetAngle);
        } else{
            currentSide = sides.blueSide;
            blueSide(targetX,targetY,targetZ, targetAngle);
        }
    }

    private void blueFront(double targetX, double targetY, double targetZ, double targetAngle){
        robotX = -33;
        robotAngle = Math.atan(Math.abs(robotY - targetY) / Math.abs(robotX - targetX)) * (180/Math.PI);
        armRotation = Math.atan(Math.abs(wristLength-targetZ) / Math.sqrt( Math.pow((robotX-targetX),2) + Math.pow((robotY-targetY), 2)) - pivotOffset) * (180/Math.PI);
        armLength = 1 / Math.cos(armRotation)*Math.sqrt( Math.pow((robotX-targetX),2) + Math.pow((robotY-targetY), 2)) - pivotOffset;
        wristTwist = robotAngle - targetAngle;
        wristRotation = 90 - armRotation;

        armLength = clipRange(armLength, armExtensionMin, armExtensionMax);
        armRotation = clipRange(armRotation, armRotationMin, armRotationMax);
        wristTwist = clipRange(wristTwist, wristTwistMin, wristTwistMax);
        wristRotation = clipRange(wristRotation, wristRotMin, wristRotMax);
    }

    private void redFront(double targetX, double targetY, double targetZ, double targetAngle){
        robotX = 33;
        robotAngle = Math.atan(Math.abs(robotY - targetY) / Math.abs(robotX - targetX)) * (180/Math.PI);
        armRotation = Math.atan(Math.abs(wristLength-targetZ) / Math.sqrt( Math.pow((robotX-targetX),2) + Math.pow((robotY-targetY), 2)) - pivotOffset) * (180/Math.PI);
        armLength = 1 / Math.cos(armRotation)*Math.sqrt( Math.pow((robotX-targetX),2) + Math.pow((robotY-targetY), 2)) - pivotOffset;
        wristTwist = robotAngle - targetAngle;
        wristRotation = 90 - armRotation;
    }

    private void redSide(double targetX, double targetY, double targetZ, double targetAngle){
        robotX = -25;
        robotAngle = Math.atan(Math.abs(robotX - targetX) / Math.abs(robotY - targetY)) * (180/Math.PI);
        armRotation = Math.atan(Math.abs(wristLength-targetZ) / Math.sqrt( Math.pow((robotY-targetY),2) + Math.pow((robotX-targetX), 2)) - pivotOffset) * (180/Math.PI);
        armLength = 1 / Math.cos(armRotation)*Math.sqrt( Math.pow((robotY-targetY),2) + Math.pow((robotX-targetX), 2)) - pivotOffset;
        wristTwist = robotAngle - targetAngle;
        wristRotation = 90 - armRotation;
    }

    private void blueSide(double targetX, double targetY, double targetZ, double targetAngle){
        robotX = 25;
        robotAngle = Math.atan(Math.abs(robotX - targetX) / Math.abs(robotY - targetY)) * (180/Math.PI);
        armRotation = Math.atan(Math.abs(wristLength-targetZ) / Math.sqrt( Math.pow((robotY-targetY),2) + Math.pow((robotX-targetX), 2)) - pivotOffset) * (180/Math.PI);
        armLength = 1 / Math.cos(armRotation)*Math.sqrt( Math.pow((robotY-targetY),2) + Math.pow((robotX-targetX), 2)) - pivotOffset;
        wristTwist = robotAngle - targetAngle;
        wristRotation = 90 - armRotation;
    }

    public double getArmLength() {
        return armLength;
    }

    public double getArmRotation() {
        return armRotation;
    }

    public double getRobotX(){
        return robotX;
    }

    public double getRobotY(){
        return robotY;
    }

    public double getRobotAngle(){
        return robotAngle;
    }

    public double getWristRotation(){
        return wristRotation;
    }

    public double getWristTwist(){
        return wristTwist;
    }

    private double clipRange(double input, double min, double max){
        if (input < min) input = min;
        if (input > max) input = max;
        return input;
    }

}
