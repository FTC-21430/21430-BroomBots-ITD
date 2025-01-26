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
    private double armExtension;
    
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
    private final double ARM_EXTENSION_MIN = 0.0, ARM_EXTENSION_MAX = 19.5;

    private final double ARM_ROTATION_MIN = 8.0, ARM_ROTATION_MAX = 180.0;

    private final double ELBOW_ROTATION_MIN = -180.0, ELBOW_ROTATION_MAX = 180.0;

    private final double TWIST_MIN = 0, TWIST_MAX = 180.0;
    
    //the distance between the center of the robot and the center of the claw on the x axis
    public final double ELBOW_OFFSET = 4.0;
    
    // the length in inches of the non-extending shaft.
    private final double FOREARM_LENGTH = 11.18;

    // how far the pivot point of the arm is away from the center of the robot.
    private final double PIVOT_OFFSET = 2.55;

    // the distance between the bottom of the wheels to the center of the arm pivot point
    private final double CHASSIS_HEIGHT = 5.73;

    private final double TUBE_LENGTH = 16.75;

    // how close we can be to a sample and still pick it up without moving back = ~21.5
    private final double MIN_H = PIVOT_OFFSET + Math.hypot(TUBE_LENGTH, FOREARM_LENGTH + 1.5 - CHASSIS_HEIGHT);
    
    // how far away a sample can be without us breaking the expansion limit
    private final double MAX_H = 26; // inches from center the of the robot
    
    
    // the constructor for this class.... that needs to do nothing.. yup
    public InverseKinematics() {
        /* Suggestion: Have constructor take robot position and sample position as arguments
        move the calculate kinematics functionality to the constructor.
        (But not the twist)
         */
    }

    public boolean verifyLength(double Rx,double Ry,double Tx,double Ty){
        double h = Math.hypot(Tx-Rx,Ty-Ry);

      return h <= MAX_H;

    }
    /**
     * @param currentX the current robot x position used to figure out the side.
     * @param currentY the current robot Y position used to figure out the side.
     * @param sampleX the x position of the sample in field coordinates
     * @param sampleY the y position of the sample in field coordinates
     * @param targetZ how far off the ground should the inside of the claw move to.
     * @param sampleAngle the angle of the sample on the z axis, this cannot pickup samples that are not
     *                   laying flat on the floor
     *      * 1) Find robotAngle: angle for elbow-offset-adjusted robot to face sample
     *      * 2) Adjust robot x,y for elbow offset
     *      * 3) Find target Robot xy based on least movement to reach sample
     *      * 4) Find arm arm rotation and arm extension, based on distance to sample and target height
     *      * 5) tbd - Find twist
     */
    public void calculateKinematics(double currentX, double currentY, double sampleX, double sampleY, double targetZ, double sampleAngle) {
   // 1) Find robotAngle: angle for elbow-offset-adjusted robot to face sample
        final double COORDINATE_ADJUSTMENT = 90.0;

        double a = sampleY - currentY; // adjacent side
        double o = sampleX - currentX; // opposite side

        // robot heading 0 deg is +y axis !!  subtracting 90 also means range of theta is -270 to +90
        robotAngle = Math.toDegrees(Math.atan2(a,o))-COORDINATE_ADJUSTMENT;

     // 2) Adjust robot x,y for elbow offset
        robotX = currentX - ELBOW_OFFSET * Math.cos(Math.toRadians(robotAngle));
        robotY = currentY - ELBOW_OFFSET * Math.sin(Math.toRadians(robotAngle));

     // 3) Find target Robot xy based on least movement to reach sample
        double h = Math.hypot(sampleX-robotX,sampleY-robotY);
        
        if (h < MIN_H){
            double lengthError = MIN_H - h;
            robotX -= lengthError * Math.cos(robotAngle);
            robotY -= lengthError * Math.sin(robotAngle);
            h=MIN_H;
        }

        if (h > MAX_H){
            double lengthError = h - MAX_H;
            robotX += lengthError * Math.cos(robotAngle);
            robotY += lengthError * Math.sin(robotAngle);
            h=MAX_H;
        }

    // 4) Find arm arm rotation and arm extension, based on distance to sample and target height
        // adjacent: horizontal distance from pivot to target
        double elbowA = h - PIVOT_OFFSET;
        // opposite: vertical distance from chassis to elbow height
        double elbowO = FOREARM_LENGTH + targetZ - CHASSIS_HEIGHT;

        armRotation = Math.toDegrees(Math.atan2(elbowO,elbowA));
        // because we've already checked that distance from robot to sample
        // is >= MIN_H and <=MAX_H, armRotation and armExtension should be in range
        // but just in case ...
        armRotation = Math.max(armRotation,ARM_ROTATION_MIN);
        armRotation = Math.min(armRotation, ARM_ROTATION_MAX);

        armExtension = Math.hypot(elbowO,elbowA)- TUBE_LENGTH;
        armExtension = Math.max(armExtension, ARM_EXTENSION_MIN);
        armExtension = Math.min(armExtension, ARM_EXTENSION_MAX);

        // elbow rotation and arm rotation are complementary angles
        elbowRotation = 90 - armRotation;

        // 5) Find twist
        /* Seems like this could be separate from inverse kinematics,
        as you might have multiple options for gripping a sample
        once the claw is above the sample.
         */

        twist = sampleAngle - (-robotAngle);
    }

    // returns how much the arm should be extended
    public double getArmExtension() {
        return armExtension;
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
}
