package frc.robot.commands.hood;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Hood;
import frc.robot.util.InterpolatedTreeMap;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

public class HoodManual extends IndefiniteCommand{

    private static final double HOOD_KP = 4; 
    private static final double HOOD_KI = 0;
    private static final double HOOD_KD = 2; 
    private static final double HOOD_IZONE = 100000; 
    private static final double GRAVITY_FF = 0.05;  
    
    private double hoodPosition;

    private PIDController hoodController;
    private InterpolatedTreeMap referencePoints;
    
    public HoodManual(){
        addRequirements(Hood.getInstance());

        hoodController = new PIDController(HOOD_KP, HOOD_KI, HOOD_KD);
        hoodController.setIntegratorRange(-HOOD_IZONE, HOOD_IZONE);

        referencePoints = new InterpolatedTreeMap();
        referencePoints.put(0.17, 0.1);
        referencePoints.put(0.7, 0.2655);
        referencePoints.put(1.0, 0.354);
        referencePoints.put(1.3, 0.6195);
        referencePoints.put(1.75, 1.062);
        referencePoints.put(1.98, 1.1852);
        referencePoints.put(2.38, 1.2213);
        referencePoints.put(2.8, 1.3629);
        referencePoints.put(3.24, 1.4514);
        referencePoints.put(3.7, 1.48);
    }

    public void initialize() {
        hoodController.reset();
    }
    
    public void execute() {
        if(!Hood.isZeroed) return;

        if(Limelight.isTargetVisible()) {
            hoodPosition = referencePoints.get(Limelight.getDistance());
        }

        double controlEffort = hoodController.calculate(Hood.getInstance().getHoodPos(), hoodPosition) + GRAVITY_FF;
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, controlEffort);
        SmartDashboard.putNumber("hood pid setpoint", hoodController.getSetpoint());
        SmartDashboard.putNumber("hood pid error", hoodController.getPositionError());
        SmartDashboard.putNumber("hood pid control effort", controlEffort);
    }

    public void end(boolean interrupted) {
        hoodPosition = 0;
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, 0);
    }
}
