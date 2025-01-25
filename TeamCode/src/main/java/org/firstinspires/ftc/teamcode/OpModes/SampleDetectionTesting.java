package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Systems.Camera;

@TeleOp
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
            telemetry.addLine("SampleX: " + sampleCamera.getTempX() + ", "+ sampleCamera.getTempYaw());
            telemetry.addLine("SampleY" + sampleCamera.getTempY());
            telemetry.update();
        }
    }

}
