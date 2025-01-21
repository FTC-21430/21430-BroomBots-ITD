package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This class has the functions for the climbers
public class Climber {

    // This function is always the starting position of the climber at 0,0

    //THESE NUMBERS ARE PLACEHOLDERS! 25.4 is mm per inch|2000 is ticks per revolution|48*Math.PI is
    // mm per revolution
    private final double ticksPerInches = 364.9/4.469;

    private final double inchesPerTicks = 4.469/364.9;

    private final double minExtension = 0.0; //inches

    private final double maxExtension = 12.75;

    //TODO: tune this value
    private double startingOffsetInches = 1;

    private DcMotor leftClimberMotor;
    private DcMotor rightClimberMotor;
    // These are the two motors that control the climber and they run in parallel

    private ServoPlus leftLatch;
    private ServoPlus rightLatch;

    private DigitalChannel limitL;
    private DigitalChannel limitR;

    private boolean docking = false;

    private Telemetry telemetry;
    public Climber(HardwareMap hardwareMap, Telemetry telemetry){
        rightClimberMotor = hardwareMap.get (DcMotor.class, "rightClimberMotor");
        leftClimberMotor = hardwareMap.get(DcMotor.class, "leftClimberMotor");

        leftClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftClimberMotor.setTargetPosition(0);
        rightClimberMotor.setTargetPosition(0);

        leftClimberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightClimberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftClimberMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightClimberMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftClimberMotor.setPower(1);
        rightClimberMotor.setPower(1);

        startingPosition();

        //TODO: tune the range of motion for these new servos
        leftLatch = new ServoPlus(hardwareMap.get(Servo.class, "latchL"), 180, 0, 180);
        rightLatch = new ServoPlus(hardwareMap.get(Servo.class, "latchR"), 180, 0, 180);

        limitL = hardwareMap.get(DigitalChannel.class, "limitSwitchL");
        limitR = hardwareMap.get(DigitalChannel.class, "limitSwitchR");

        this.telemetry = telemetry;
    }

    // The motors go to their lowest extension
    public void startingPosition(){
        leftClimberMotor.setTargetPosition(0);
        rightClimberMotor.setTargetPosition(0);
    }
    public void extendTo(double inches){

        if (inches > maxExtension) {
            inches = maxExtension;
        }
        if (inches < minExtension) {
            inches = minExtension;
        }

        inches += startingOffsetInches;

        leftClimberMotor.setTargetPosition((int)(inches * ticksPerInches));
        rightClimberMotor.setTargetPosition((int)(inches * ticksPerInches));

    }

    public void dock(){
        double inches = 0;

        inches += startingOffsetInches;

        leftClimberMotor.setTargetPosition((int)(inches * ticksPerInches));
        rightClimberMotor.setTargetPosition((int)(inches * ticksPerInches));
        docking = true;
    }

    public double getCurrentExtension(){
        return ((leftClimberMotor.getCurrentPosition() + rightClimberMotor.getCurrentPosition())/2 * inchesPerTicks) + startingOffsetInches;
    }


    //TOOO: tune the values in these functions
    public void releaseLatches(){
        leftLatch.setServoPos(80);
        rightLatch.setServoPos(80);
    }

    public void lockLatches(){
        leftLatch.setServoPos(90);
        rightLatch.setServoPos(90);
    }

    public void updateClimber(){
        if (docking){
            boolean L = limitL.getState();
            boolean R = limitR.getState();

            if (!L || !R){
                if (getCurrentExtension() < 0.5){
                    if (!L) leftClimberMotor.setPower(0.6);
                    if (!R) rightClimberMotor.setPower(0.6);
                } else {
                    if (!L) leftClimberMotor.setPower(1);
                    if (!R) rightClimberMotor.setPower(1);
                }
            }

            if (L){
                leftClimberMotor.setPower(1);
                leftClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftClimberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftClimberMotor.setTargetPosition(0);
            }
            if (R) {
                rightClimberMotor.setPower(1);
                rightClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightClimberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftClimberMotor.setTargetPosition(0);
            }

            if (L && R){
                docking = false;
                lockLatches();
            }



        }
    }

}