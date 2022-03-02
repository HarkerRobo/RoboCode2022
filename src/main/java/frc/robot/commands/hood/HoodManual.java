package frc.robot.commands.hood;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Units;
import frc.robot.subsystems.Hood;
import frc.robot.util.InterpolatedTreeMap;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

public class HoodManual extends IndefiniteCommand{

    private static final double HOOD_KP = 1.0; 
    private static final double HOOD_KI = 0.8;
    private static final double HOOD_KD = 0.018539; 
    private static final double HOOD_IZONE = 100000; 
    private static final double HOOD_KS = 0.63651; 
    private static final double HOOD_KV = 0.051961; 
    
    private double hoodPosition;

    private ProfiledPIDController hoodController;
    private InterpolatedTreeMap referencePoints;
    private SimpleMotorFeedforward feedforward;
    
    public HoodManual(){
        addRequirements(Hood.getInstance());

        hoodController = new ProfiledPIDController(HOOD_KP, HOOD_KI, HOOD_KD, new Constraints(30, 15));
        hoodController.setIntegratorRange(-HOOD_IZONE, HOOD_IZONE);
        feedforward = new SimpleMotorFeedforward(HOOD_KS, HOOD_KV);

        referencePoints = new InterpolatedTreeMap();
        referencePoints.put(0.88, 10.0);
        referencePoints.put(1.12, 14.0);
        referencePoints.put(1.49, 16.0);
        referencePoints.put(1.89, 18.0);
        referencePoints.put(2.27, 21.0);
        referencePoints.put(2.67, 23.0);
    }

    public void initialize() {
        hoodController.reset(Hood.getInstance().getHoodPos());
    }
    
    public void execute() {
        if(!Hood.isZeroed) return;

        if(Limelight.isTargetVisible()) {
            hoodPosition = referencePoints.get(Limelight.getDistance());
        }
        else
            hoodPosition = 3;
        hoodPosition = SmartDashboard.getNumber("desired hood pos", 1);
        double controlEffort = hoodController.calculate(Hood.getInstance().getHoodPos(), hoodPosition);
        double feedforwardAmount = feedforward.calculate(hoodController.getSetpoint().velocity);
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, (controlEffort + feedforwardAmount)/Units.MAX_CONTROL_EFFORT);
        SmartDashboard.putNumber("hood pid goal", hoodController.getGoal().position);
        SmartDashboard.putNumber("hood pid setpoint", hoodController.getSetpoint().position);
        SmartDashboard.putNumber("hood pid error", hoodController.getPositionError());
        SmartDashboard.putNumber("hood pid control effort", controlEffort);
    }

    public void end(boolean interrupted) {
        hoodPosition = 0;
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, 0);
    }
}
