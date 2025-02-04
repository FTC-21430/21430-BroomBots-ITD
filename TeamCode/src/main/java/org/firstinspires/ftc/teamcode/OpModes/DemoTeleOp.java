package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// This is used for outreach. It doesn't have all of the functions of normal teleops.
@TeleOp
public class DemoTeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, false);
    }
}
