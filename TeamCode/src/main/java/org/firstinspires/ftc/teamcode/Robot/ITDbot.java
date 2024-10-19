package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ITDbot extends Robot {
    //TODO tune these numbers for ITDbot, need to be updated for Into the deep
    public static double derivativeConstantAngleDef = 0.0015;
    public static double proportionalConstantAngleDef = 0.02;
    public void Init(HardwareMap hardwareMap, Telemetry telemetry){
        super.init(hardwareMap, telemetry,0, 0, 0);
        derivativeConstantAngle = derivativeConstantAngleDef;
        proportionalConstantAngle = proportionalConstantAngleDef;
    }
}
