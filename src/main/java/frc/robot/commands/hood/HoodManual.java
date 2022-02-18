package frc.robot.commands.hood;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Hood;
import harkerrobolib.commands.IndefiniteCommand;

public class HoodManual extends IndefiniteCommand{

    public static final double HOOD_KP = 4; //change
    public static final double HOOD_KI = 0;//0.2; //change
    public static final double HOOD_KD = 2;//0.4; //change
    public static final double HOOD_IZONE = 100000; //change
    public static final double HOOD_PID_TOLERANCE = 0.01; //change
    private static PIDController hoodController;

    
    public HoodManual(){
        addRequirements(Hood.getInstance());
        hoodController = new PIDController(HOOD_KP, HOOD_KI, HOOD_KD);
        hoodController.setIntegratorRange(-HOOD_IZONE, HOOD_IZONE);
    }

    public void initialize() {
        hoodController.reset();
    }
    
    public void execute() {
        if(!Hood.isZeroed) return;
        double controlEffort = hoodController.calculate(Hood.getInstance().getHoodPos(), 
        Hood.getInstance().getMaxHoodPos() * SmartDashboard.getNumber("desired hood pos", 0)) + 0.05;
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, controlEffort);
        SmartDashboard.putNumber("hood pid setpoint", hoodController.getSetpoint());
        SmartDashboard.putNumber("hood pid error", hoodController.getPositionError());
        SmartDashboard.putNumber("hood pid control effort", controlEffort);
    }

    public void end(boolean interrupted) {
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, 0);
    }
}
