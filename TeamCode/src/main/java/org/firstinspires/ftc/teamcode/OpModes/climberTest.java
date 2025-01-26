package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class climberTest extends BaseTeleOp {

    private boolean climbingOld = false;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true);
        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.cross){
                robot.climber.initClimber();
            }

            if (gamepad1.right_trigger >= 0.6){
                robot.climber.extendTo(12.5);
            } else{
                robot.climber.extendTo(0.0);
            }
            climbingOld = gamepad1.right_trigger >= 0.6;
            robot.climber.updateClimber();

            if (gamepad1.left_trigger > 0.6){
                robot.climber.releaseLatches();
            } else if (gamepad1.left_bumper) {
                robot.climber.lockLatches();
            }


            telemetry.addData("climberPosition", robot.climber.getCurrentExtension());
            telemetry.addData("triggerValue", gamepad1.right_trigger);
            telemetry.update();
            }
        }
    }

