package org.firstinspires.ftc.teamcode.Resources;

// this class is just for figuring out how the robot should be positioned when intaking from the submersible,
// this includes both the chassis and arm.

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 *
 *   |   o (x`, y`, h`)
 *   |
 * __|__
 * |   |
 * |___| (x, y, h)
 *
 * dx = x`-x
 * dy = y`-y
 * h = atan(dy,dx)
 *
 *      /|
 *     /b|
 *  d /  | w
 *   /a  |
 * O---O   h
 *
 * d = math
 * a = math
 * b = ...
 * ...
 */
public class InverseKinematics {

    // how much the arm extension should be out
    private double armLength;
    
    // the target rotation of the arm pivot
    private double armRotation;
    
    // how much the wrist is angled, this should always have the wrist perpendicular with the field floor.
    private double elbowRotation;
    
    // how much the twist should rotated to align with the sample's orientation
    private double twist;
    
    // the target X and Y position the robot should go to.
    private double robotX, robotY;

    // the target rotation the robot should turn to.
    private double robotAngle;


    //Todo tune these values correctly
    
    // the mechanical limits of the mechanisms so that you cannot pass a value that would break the robot.
    private final double armExtensionMax  = 19.5, armExtensionMin  = 0.0;

    private final double armRotationMin   = 8.0, armRotationMax   = 180.0;

    private final double elbowRotationMin = -180.0, elbowRotationMax = 180.0;

    private final double twistMin         = 0, twistMax         = 180.0;
    
    //the distance between the center of the robot and the center of the claw on the x axis
    private final double elbowOffsetX = 4;
    
    // the length in inches of the non-extending shaft.
    private final double elbowLength = 11.18;

    // how far the pivot point of the arm is away from the center of the robot.
    private final double pivotOffset = 5.25;

    // the distance between the bottom of the wheels to the center of the arm pivot point
    private final double chassisHeight = 5.73;
    
    

    private final double tubeLength = 16.75;

    // how close we can be to a sample and still pick it up without moving back = ~21.5
    private final double minL = pivotOffset + Math.sqrt(tubeLength*tubeLength- Math.pow(elbowLength + 1.5 - chassisHeight,2));
    
    // how far away a sample can be without us breaking the expansion limit
    private final double maxL = 26; // inches from center the of the robot
    
    
    // the constructor for this class.... that needs to do nothing.. yup
    public InverseKinematics() {

    }

    //TODO: re-comment main code as I did it wrong the first time - Tobin

    
    
    public boolean verifyLength(double Rx,double Ry,double Tx,double Ty){
        double l = calculateDistance(Tx,Ty,Rx,Ry)-pivotOffset;
      
      return !(l + pivotOffset > maxL);
//        !(l < minL) &&
    }
    /**
     * The function you call from the main code to calculate the position of the robot.
     * This function decides which side you are trying to grab from then calls the corresponding method
     * @param xCurrent the current robot x position used to figure out the side.
     * @param yCurrent the current robot Y position used to figure out the side.
     * @param targetX the x position of the sample in field coordinates
     * @param targetY the y position of the sample in field coordinates
     * @param targetZ how far off the ground should the inside of the claw move to.
     * @param targetAngle the angle of the sample on the z axis, this cannot pickup samples that are not
     *                   laying flat on the floor
     */
    public void calculateKinematics(double xCurrent, double yCurrent, double targetX, double targetY, double targetZ, double targetAngle) {
        robotX = xCurrent;
        robotY = yCurrent;
        double l = calculateDistance(targetX,targetY,xCurrent,yCurrent)-pivotOffset;
        
        robotAngle = Math.atan2((targetY- yCurrent),(targetX-xCurrent)) * (180/Math.PI) - 90;
        // subtracted by 90 degrees because of the weird definition of our coordinate system compared to the worlds standards, they should fix theirs
        // robot heading 0 deg is +y axis !!  subtracting 90 also means range of theta is -270 to +90
        
        double elbowX = -elbowOffsetX * Math.cos(-AngleUnit.DEGREES.toRadians(robotAngle)) + l * Math.sin(-AngleUnit.DEGREES.toRadians(robotAngle));
        double elbowY = elbowOffsetX * Math.sin(-AngleUnit.DEGREES.toRadians(robotAngle)) + l * Math.cos(-AngleUnit.DEGREES.toRadians(robotAngle));
        
        robotX -= (elbowX - targetX)/2;
        robotY += elbowY - targetY;
        
        l = calculateDistance(targetX,targetY,xCurrent,yCurrent)-pivotOffset;
        
        double h = elbowLength - targetZ + chassisHeight;
        armRotation = Math.atan2(h,l) * (180/Math.PI);
        armLength = Math.sqrt(l*l + h*h)-tubeLength;
        elbowRotation = 90 - armRotation;
        
        twist = targetAngle - (-robotAngle);
        
        if (armRotation < armRotationMin) armRotation = armRotationMin;
        if (armRotation > armRotationMax) armRotation = armRotationMax;
        
        if (armLength < armExtensionMin) armLength = armExtensionMin;
        if (armLength > armExtensionMax) armLength = armExtensionMax;
        
    }
    
    public static double calculateDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
    // returns how much the arm should be extended
    public double getArmLength() {
        return armLength;
    }

    // returns how much the arm should be rotated
    public double getArmRotation() {
        return armRotation;
    }

    // returns robot X
    public double getRobotX(){
        return robotX;
    }

    // returns robot Y
    public double getRobotY(){
        return robotY;
    }

    // returns robot angle
    public double getRobotAngle(){
        return robotAngle;
    }

    // returns the target rotation for the elbow
    public double getElbowRotation(){
        return elbowRotation;
    }

    // returns the target twist for the twist
    public double getTwist(){
        return twist;
    }
    
    // clips the values so the mechanisms do not exceed the mechanical limits
    private double clipRange(double input, double min, double max){
        if (input < min) input = min;
        if (input > max) input = max;
        return input;
    }

}
