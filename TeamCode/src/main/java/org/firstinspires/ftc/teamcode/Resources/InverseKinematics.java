package org.firstinspires.ftc.teamcode.Resources;

// this class is just for figuring out how the robot should be positioned when intaking from the submersible,
// this includes both the chassis and arm.

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
    private final double armExtensionMax  = 1.0, armExtensionMin  = 1.0;

    private final double armRotationMin   = 1.0, armRotationMax   = 1.0;

    private final double elbowRotationMin = 1.0, elbowRotationMax = 1.0;

    private final double twistMin         = 1.0, twistMax         = 1.0;


// the different valid sides of the Submersible that can you go to.
    private enum sides {
        redFront, // from red chambers
        redSide, // from red rungs
        blueFront, // from blue chambers
        blueSide // from blue rungs
    }

    // the current side we are picking up from
    private sides currentSide;
    


    //TODO these values are from CAD, tune these more accurate to the real robot
    
    // the length in inches of the non-extending shaft.
    private final double Length = 8.95;

    // how far the pivot point of the arm is away from the center of the robot.
    private final double pivotOffset = 5.55;

    // the distance between the bottom of the wheels to the center of the arm pivot point
    private final double chassisHeight = 6.31;

    private final double tubeLength = 13.16;

    
    // the constructor for this class.... that needs to do nothing.. yup
    public InverseKinematics() {

    }

    //TODO: re-comment main code as I did it wrong the first time - Tobin

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

       if (xCurrent > 14){
           robotAngle = Math.acos((targetX-robotX)/(targetY-robotY)) * (180/Math.PI);
       }
       else if (robotX < -14){
           robotAngle = Math.asin((targetX-robotX)/(targetY-robotY)) * (180/Math.PI);
       }
       else if(robotY < 0){
           robotAngle = Math.atan((targetX-robotX)/(targetY-robotY)) * (180/Math.PI) * -1;
       }
       else{
           robotAngle = Math.atan((targetX-robotX)/(targetY-robotY)) * (180/Math.PI) * -1 + 180;
       }
        double l = Math.([robotX,robotY],[targetX,targetY])-pivotOffset;
        double h = Length - targetZ;
        armRotation = Math.atan(h/l) * (180/Math.PI);
        armLength = Math.sqrt(l*l + h*h)-tubeLength;
        alpha = 90 - phi

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
