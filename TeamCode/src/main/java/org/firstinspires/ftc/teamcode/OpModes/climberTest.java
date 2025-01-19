package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class climberTest extends BaseTeleOp {

    private boolean climbingOld = false;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true);
        waitForStart();
        robot.climber.releaseLatches();
        while (opModeIsActive()) {

            if (gamepad1.right_trigger >= 0.6){
                robot.climber.extendTo(12.5);
            } else if(climbingOld){
                robot.climber.dock();
            }
            climbingOld = gamepad1.right_trigger >= 0.6;
            robot.climber.updateClimber();

            telemetry.addData("climberPosition", robot.climber.getCurrentExtension());
            telemetry.addData("triggerValue", gamepad1.right_trigger);
            telemetry.update();
            }
        }
    }

