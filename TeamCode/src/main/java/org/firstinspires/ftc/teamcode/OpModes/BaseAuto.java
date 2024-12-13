package org.firstinspires.ftc.teamcode.OpModes;
// This class will have the autonomous functions applicable to every auto. All autos extend BaseAuto.
abstract public class BaseAuto extends org.firstinspires.ftc.teamcode.OpModes.GeneralOpMode {
    public static double PconstantFast= 0.15;
    public static double IconstantFast= 0.1;
    public static double DConstantFast= 0.02;
    public static double speedMultplierFast=1;

    public static double PconstantSlow= 0.5;
    public static double IconstantSlow= 1;
    public static double DConstantSlow= 0.02;
    public static double speedMultplierSlow=0.3;
    
    public void setAutoSpeedSlow(){
        robot.driveTrain.setSpeedMultiplier(speedMultplierSlow);
        robot.pathFollowing.xPID.updateConstants(PconstantSlow, IconstantSlow  , DConstantSlow);
        robot.pathFollowing.yPID.updateConstants(PconstantSlow, IconstantSlow, DConstantSlow);
    }
    public void setAutoSpeedFast(){
        robot.driveTrain.setSpeedMultiplier(speedMultplierFast);
        robot.pathFollowing.xPID.updateConstants(PconstantFast, IconstantFast  , DConstantFast);
        robot.pathFollowing.yPID.updateConstants(PconstantFast, IconstantFast, DConstantFast);
    }
    
}
