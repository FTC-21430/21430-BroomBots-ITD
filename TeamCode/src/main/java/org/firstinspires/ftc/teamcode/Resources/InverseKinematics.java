package org.firstinspires.ftc.teamcode.Resources;

// this class is just for figuring out how the robot should be positioned when intaking from the submersible,
// this includes both the chassis and arm.

import com.acmerobotics.dashboard.config.Config;

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
@Config
public class InverseKinematics {


    // how much the arm extension should be out
    private double armExtension;
    
    // the target rotation of the arm pivot
    private double armRotation;
    
    // how much the wrist is angled, this should always have the wrist perpendicular with the field floor.
    private double elbowRotation = 0;
    
    // how much the twist should rotated to align with the sample's orientation
    private double twist;
    
    // the target X and Y position the robot should go to.
    private double robotX, robotY;

    // the target rotation the robot should turn to.
    private double robotAngle;
    
    // the mechanical limits of the mechanisms so that you cannot pass a value that would break the robot.
    private final double ARM_EXTENSION_MIN = 0.0, ARM_EXTENSION_MAX = 19.5;

    private final double ARM_ROTATION_MIN = 11.0, ARM_ROTATION_MAX = 170.0;


    //TODO figure out what to do with these
    private final double ELBOW_ROTATION_MIN = -180.0, ELBOW_ROTATION_MAX = 180.0;

    private final double TWIST_MIN = 0, TWIST_MAX = 180.0;
    
    //the distance between the center of the robot and the center of the claw on the x axis
    public static double ELBOW_OFFSET = 3;
    // old number: 4.8 in
    
    // the length in inches of the non-extending shaft.
    public static double FOREARM_LENGTH = 11.45;

    public static double GRABBERS_OFFSET = 0.8;

    // how far the pivot point of the arm is away from the center of the robot.
    public static double PIVOT_OFFSET = 4.375;

    // the distance between the bottom of the wheels to the center of the arm pivot point
    public static double CHASSIS_HEIGHT = 5.5;

    public static double TUBE_LENGTH = 16.75;

    // how close we can be to a sample and still pick it up without moving back = ~21.5
    private final double MIN_H = PIVOT_OFFSET + Math.hypot(TUBE_LENGTH, FOREARM_LENGTH + 1.5 - CHASSIS_HEIGHT);
    
    // how far away a sample can be without us breaking the expansion limit
    private final double MAX_H = 24.9; // inches from center the of the robot
    
    
    // the constructor for this class.... that needs to do nothing.. yup
    public InverseKinematics() {
        /* Suggestion: Have constructor take robot position and sample position as arguments
        move the calculate kinematics functionality to the constructor.
        (But not the twist)
         */
    }

    public boolean verifyLength(double Rx,double Ry,double Tx,double Ty){
        double h = Math.hypot(Tx-Rx,Ty-Ry);

      return h <= MAX_H && h >= 4;

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
    public boolean calculateKinematics(double currentX, double currentY, double sampleX, double sampleY, double targetZ, double sampleAngle,double robotRotation) {
        boolean passed = true;

        if (!verifyLength(currentX,currentY,sampleX,sampleY)){
            passed = false;
        }


        sampleAngle = sampleAngle * (Math.PI/180);
        double clawAddedY = GRABBERS_OFFSET * -Math.sin(sampleAngle);
        double sampleDistance = Math.hypot(sampleX - currentX, sampleY - currentY);
        double topDownArmLength = Math.sqrt(Math.pow(sampleDistance,2) - Math.pow((GRABBERS_OFFSET * Math.cos(sampleAngle)) + ELBOW_OFFSET,2)) - (clawAddedY + PIVOT_OFFSET);
        double angle1 = Math.acos((clawAddedY + topDownArmLength + PIVOT_OFFSET) / sampleDistance);
        double angle2 = Math.asin((currentX - sampleX)/sampleDistance);
        robotAngle = (angle1 - angle2) * -(180/Math.PI);
        armRotation = Math.atan2((FOREARM_LENGTH + targetZ) - CHASSIS_HEIGHT,topDownArmLength) * (180/Math.PI);
        elbowRotation = 90 - armRotation;
        armExtension = Math.sqrt(Math.pow(topDownArmLength,2)+ Math.pow((FOREARM_LENGTH+targetZ) - CHASSIS_HEIGHT,2)) - TUBE_LENGTH;
        sampleAngle = sampleAngle * (180/Math.PI);
        twist = sampleAngle - robotAngle;

        if (armRotation > ARM_ROTATION_MAX || armRotation < ARM_ROTATION_MIN){
            passed = false;
        }
        if (armExtension > ARM_EXTENSION_MAX || armExtension < ARM_EXTENSION_MIN){
            passed = false;
        }
        if (elbowRotation <30){
            passed = false;
        }


        return passed;
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
