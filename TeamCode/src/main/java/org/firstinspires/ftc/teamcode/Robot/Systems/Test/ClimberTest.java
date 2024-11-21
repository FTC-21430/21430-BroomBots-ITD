package org.firstinspires.ftc.teamcode.Robot.Systems.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.Systems.Climber;
@TeleOp
public class ClimberTest extends LinearOpMode {

   @Override
   public void runOpMode() throws InterruptedException {
      Climber climber= new Climber(hardwareMap);
      waitForStart();
      while(opModeIsActive()){
         if (gamepad1.left_bumper){
         climber.climberAssent();
         }else {
            climber.startingPosition();
         }
      }
   }
}
