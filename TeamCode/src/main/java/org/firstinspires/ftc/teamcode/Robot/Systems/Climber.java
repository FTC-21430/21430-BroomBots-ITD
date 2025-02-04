package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This class has the functions for the climbers
@Config
public class Climber {

    // This function is always the starting position of the climber at 0,0

    //THESE NUMBERS ARE PLACEHOLDERS! 25.4 is mm per inch|2000 is ticks per revolution|48*Math.PI is
    // mm per revolution
    private final double ticksPerInches = 364.9/4.469;

    private final double inchesPerTicks = 4.469/364.9;

    private final double minExtension = 0.0; //inches

    private final double maxExtension = 12.75;

    public static double leftLatchZero = 63;
    public static double rightLatchZero = 291;

    public static double latchReleaseMovementRight = 52;
    public static double latchReleaseMovementLeft = 57;

    private DcMotor leftClimberMotor;
    private DcMotor rightClimberMotor;
    // These are the two motors that control the climber and they run in parallel

    private ServoPlus leftLatch;
    private ServoPlus rightLatch;

    private DigitalChannel limitL;
    private DigitalChannel limitR;

    private boolean docking = false;

    private Telemetry telemetry;

    private HardwareMap hardwareMap;


    private boolean initilized = false;
    public Climber(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }


    public void initClimber(){
        initilized = true;

        rightClimberMotor = hardwareMap.get (DcMotor.class, "rightClimberMotor");
        leftClimberMotor = hardwareMap.get(DcMotor.class, "leftClimberMotor");

        leftClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightClimberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftClimberMotor.setTargetPosition(0);
        rightClimberMotor.setTargetPosition(0);

        leftClimberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightClimberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftClimberMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightClimberMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftClimberMotor.setPower(1);
        rightClimberMotor.setPower(1);


        leftLatch = new ServoPlus(hardwareMap.get(Servo.class, "latchL"), 295, 0, 295);
        rightLatch = new ServoPlus(hardwareMap.get(Servo.class, "latchR"), 295,0,295);

//        releaseLatches();
        lockLatches();


    }
    // The motors go to their lowest extension
    public void startingPosition(){
        leftClimberMotor.setTargetPosition(0);
        rightClimberMotor.setTargetPosition(0);
    }
    public void extendTo(double inches){

        if (initilized) {


            if (inches > maxExtension) {
                inches = maxExtension;
            }
            if (inches < minExtension) {
                inches = minExtension;
            }

            leftClimberMotor.setTargetPosition((int) (inches * ticksPerInches));
            rightClimberMotor.setTargetPosition((int) (inches * ticksPerInches));
        }
    }

    public void dock(){

        if (initilized){
        double inches = 0;
        leftClimberMotor.setTargetPosition((int)(inches * ticksPerInches));
        rightClimberMotor.setTargetPosition((int)(inches * ticksPerInches));
        }
    }

    public double getCurrentExtension(){
        if (initilized) {
            return ((leftClimberMotor.getCurrentPosition() + rightClimberMotor.getCurrentPosition()) / 2 * inchesPerTicks);
        }else{
            return -1.111;
        }
    }


    //TOOO: tune the values in these functions
    public void releaseLatches(){
        if (initilized) {
            leftLatch.setServoPos(leftLatchZero + latchReleaseMovementLeft);
            rightLatch.setServoPos(rightLatchZero - latchReleaseMovementRight);
        }
    }

    public void lockLatches(){
        if (initilized) {
            leftLatch.setServoPos(leftLatchZero);
            rightLatch.setServoPos(rightLatchZero);
        }
    }

    public void updateClimber(){
      if (initilized) {


      }
    }

    public boolean getIfInitilized(){
        return initilized;
    }

}