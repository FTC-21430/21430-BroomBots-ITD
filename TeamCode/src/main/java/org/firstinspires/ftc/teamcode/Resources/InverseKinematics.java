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
    private double wristRotation;
    
    // how much the wrist should rotated to align with the sample's orientation
    private double wristTwist;
    
    // the target X and Y position the robot should go to.
    private double robotX, robotY;

    // the target rotation the robot should turn to.
    private double robotAngle;


    //Todo tune these values correctly
    
    // the mechanical limits of the mechanisms so that you cannot pass a value that would break the robot.
    private final double armExtensionMax = 1.0, armExtensionMin = 1.0;

    private final double armRotationMin = 1.0, armRotationMax = 1.0;

    private final double wristRotMin = 1.0, wristRotMax = 1.0;

    private final double wristTwistMin = 1.0, wristTwistMax = 1.0;


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
    
    // the length in inches of the wrist.
    private final double wristLength = 8.95;

    // how far the pivot point of the arm is away from the center of the robot.
    private final double pivotOffset = 5.55;

    // the distance between the bottom of the wheels to the center of the arm pivot point
    private final double chassisHeight = 6.31;
    
    // the constructor for this class.... that needs to do nothing.. yup
    public InverseKinematics() {

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
        //covers any side case to ensure the robot is the correct distance away from the edge of the submersible.
        // depending on which side one of these will be overridden.
        robotX = xCurrent;
        robotY = yCurrent;
        
        // just a simple if else tree to identify which side to go to.
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
    
    /**
     * calculates the rotations/positions of everything for the blue front side of the submersible
     * @param targetX x of the sample, field coordinates and in inches
     * @param targetY y of the sample, field coordinates and in inches
     * @param targetZ how far the center of the claw should be from the floor of the field
     * @param targetAngle the angle of the sample in degrees
     */
    private void blueFront(double targetX, double targetY, double targetZ, double targetAngle){
        // sets how far the robot should be from the edge of the submersible
        robotX = -33;
        
        // calculates how much the robot should be rotated to face the target sample,
        // this originally returns in radians so we convert it to degrees
        robotAngle = Math.atan(Math.abs(robotY - targetY) / Math.abs(robotX - targetX)) * (180/Math.PI);
    
        /**
         *   calculates how much the pivot should to rotated from the base of the robot using tan^-1 and
         *          the distance of the the sample from the pivot and the sample along with the wrist length as the opposite length.
         *          this assumes that the wrist will always be perpendicular to the field floor making this a right triangle
         *          will get converted to degrees once we are done with it in rads.
         */
      
        armRotation = Math.atan(Math.abs(wristLength-(chassisHeight - targetZ)) / Math.sqrt( Math.pow((robotX-targetX),2) + Math.pow((robotY-targetY), 2)) - pivotOffset);
        
        
        // calculates what the distance the arm should extend in inches using the arm rotation
        armLength = 1 / Math.cos(armRotation)*Math.sqrt( Math.pow((robotX-targetX),2) + Math.pow((robotY-targetY), 2)) - pivotOffset;
        
        // converts armRotation into degrees
        armRotation *= (180/Math.PI);
        
        // calculates how much the wrist should twist to get all the way aligned with the sample
        wristTwist = robotAngle - targetAngle;
        
        //calculates the wrist rotation to make it perpendicular with the field floor using the rule that
        // all the angles of a triangle add up to 180.
        wristRotation = 90 - armRotation;

        // ensures that the range of values is legal.
        armLength = clipRange(armLength, armExtensionMin, armExtensionMax);
        armRotation = clipRange(armRotation, armRotationMin, armRotationMax);
        wristTwist = clipRange(wristTwist, wristTwistMin, wristTwistMax);
        wristRotation = clipRange(wristRotation, wristRotMin, wristRotMax);
    }
    
    /**
     * calculates the rotations/positions of everything for the blue front side of the submersible
     * @param targetX x of the sample, field coordinates and in inches
     * @param targetY y of the sample, field coordinates and in inches
     * @param targetZ how far the center of the claw should be from the floor of the field
     * @param targetAngle the angle of the sample in degrees
     */
    private void redFront(double targetX, double targetY, double targetZ, double targetAngle){
        // sets how far the robot should be from the edge of the submersible
        robotX = 33;
    
        // calculates how much the robot should be rotated to face the target sample,
        // this originally returns in radians so we convert it to degrees
        robotAngle = Math.atan(Math.abs(robotY - targetY) / Math.abs(robotX - targetX)) * (180/Math.PI);
    
        //calculates how much the pivot should to rotated from the base of the robot using tan^-1 and
        // the distance of the the sample from the pivot and the sample along with the wrist length as the opposite length.
        // this assumes that the wrist will always be perpendicular to the field floor making this a right triangle
        // will get converted to degrees once we are done with it in rads.
        armRotation = Math.atan(Math.abs(wristLength-(chassisHeight - targetZ)) / Math.sqrt( Math.pow((robotX-targetX),2) + Math.pow((robotY-targetY), 2)) - pivotOffset) * (180/Math.PI);
       
        // calculates what the distance the arm should extend in inches using the arm rotation
        armLength = 1 / Math.cos(armRotation)*Math.sqrt( Math.pow((robotX-targetX),2) + Math.pow((robotY-targetY), 2)) - pivotOffset;
    
        // calculates how much the wrist should twist to get all the way aligned with the sample
        wristTwist = robotAngle - targetAngle;
    
        //calculates the wrist rotation to make it perpendicular with the field floor using the rule that
        // all the angles of a triangle add up to 180.
        wristRotation = 90 - armRotation;
    }
    
    /**
     * calculates the rotations/positions of everything for the blue front side of the submersible
     * @param targetX x of the sample, field coordinates and in inches
     * @param targetY y of the sample, field coordinates and in inches
     * @param targetZ how far the center of the claw should be from the floor of the field
     * @param targetAngle the angle of the sample in degrees
     */
    private void redSide(double targetX, double targetY, double targetZ, double targetAngle){
        // sets how far the robot should be from the edge of the submersible
        robotX = -25;
    
        // calculates how much the robot should be rotated to face the target sample,
        // this originally returns in radians so we convert it to degrees
        robotAngle = Math.atan(Math.abs(robotX - targetX) / Math.abs(robotY - targetY)) * (180/Math.PI);
    
        //calculates how much the pivot should to rotated from the base of the robot using tan^-1 and
        // the distance of the the sample from the pivot and the sample along with the wrist length as the opposite length.
        // this assumes that the wrist will always be perpendicular to the field floor making this a right triangle
        // will get converted to degrees once we are done with it in rads.
        armRotation = Math.atan(Math.abs(wristLength-(chassisHeight - targetZ)) / Math.sqrt( Math.pow((robotY-targetY),2) + Math.pow((robotX-targetX), 2)) - pivotOffset) * (180/Math.PI);
    
        // calculates what the distance the arm should extend in inches using the arm rotation
        armLength = 1 / Math.cos(armRotation)*Math.sqrt( Math.pow((robotY-targetY),2) + Math.pow((robotX-targetX), 2)) - pivotOffset;
    
        // calculates how much the wrist should twist to get all the way aligned with the sample
        wristTwist = robotAngle - targetAngle;
    
        //calculates the wrist rotation to make it perpendicular with the field floor using the rule that
        // all the angles of a triangle add up to 180.
        wristRotation = 90 - armRotation;
    }
    
    /**
     * calculates the rotations/positions of everything for the blue front side of the submersible
     * @param targetX x of the sample, field coordinates and in inches
     * @param targetY y of the sample, field coordinates and in inches
     * @param targetZ how far the center of the claw should be from the floor of the field
     * @param targetAngle the angle of the sample in degrees
     */
    private void blueSide(double targetX, double targetY, double targetZ, double targetAngle){
        // sets how far the robot should be from the edge of the submersible
        robotX = 25;
    
        // calculates how much the robot should be rotated to face the target sample,
        // this originally returns in radians so we convert it to degrees
        robotAngle = Math.atan(Math.abs(robotX - targetX) / Math.abs(robotY - targetY)) * (180/Math.PI);
    
        //calculates how much the pivot should to rotated from the base of the robot using tan^-1 and
        // the distance of the the sample from the pivot and the sample along with the wrist length as the opposite length.
        // this assumes that the wrist will always be perpendicular to the field floor making this a right triangle
        // will get converted to degrees once we are done with it in rads.
        armRotation = Math.atan(Math.abs(wristLength-(chassisHeight - targetZ)) / Math.sqrt( Math.pow((robotY-targetY),2) + Math.pow((robotX-targetX), 2)) - pivotOffset) * (180/Math.PI);
    
        // calculates what the distance the arm should extend in inches using the arm rotation
        armLength = 1 / Math.cos(armRotation)*Math.sqrt( Math.pow((robotY-targetY),2) + Math.pow((robotX-targetX), 2)) - pivotOffset;
       
        // calculates how much the wrist should twist to get all the way aligned with the sample
        wristTwist = robotAngle - targetAngle;
    
        //calculates the wrist rotation to make it perpendicular with the field floor using the rule that
        // all the angles of a triangle add up to 180.
        wristRotation = 90 - armRotation;
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

    // returns the target rotation for the wrist
    public double getWristRotation(){
        return wristRotation;
    }

    // returns the target twist for the wrist
    public double getWristTwist(){
        return wristTwist;
    }
    
    // clips the values so the mechanisms do not exceed the mechanical limits
    private double clipRange(double input, double min, double max){
        if (input < min) input = min;
        if (input > max) input = max;
        return input;
    }

}
