package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Systems.Camera;

public class SampleDetectionTesting extends LinearOpMode {


    private Camera sampleCamera;

    @Override
    public void runOpMode() throws InterruptedException {


        sampleCamera = new Camera(hardwareMap, telemetry);

        sampleCamera.findYellowSample();

        while(opModeInInit()){
            telemetry.addLine("Sample: " + sampleCamera.getTempX() + ", " + sampleCamera.getTempY() + ", "+ sampleCamera.getTempYaw());
            telemetry.update();
        }

        while(opModeIsActive()){
            telemetry.addLine("Sample: " + sampleCamera.getTempX() + ", " + sampleCamera.getTempY() + ", "+ sampleCamera.getTempYaw());
            telemetry.update();
        }
    }

}
