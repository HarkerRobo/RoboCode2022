
package frc.robot.commands.hood;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Units;
import frc.robot.subsystems.Hood;
import frc.robot.util.InterpolatedTreeMap;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

public class HoodManual extends IndefiniteCommand{

    public static final double HOOD_KP = 0.107; 
    public static final double HOOD_KI = 0.017;//0.004;
    public static final double HOOD_KD = 0;//0.0020817; 
    public static final double HOOD_IZONE = 0.2; 
    public static final double HOOD_KS = 0.45;//0.7; 

    public static final double HOOD_KV = 0.049112; 
    public static final double HOOD_KA = 0.0006622;
    public static final double HOOD_KG = 0.087132;
    
    private double hoodPosition;

    private ProfiledPIDController hoodController;
    private InterpolatedTreeMap referencePoints;
    private SimpleMotorFeedforward feedforward;
    
    public HoodManual(){
        addRequirements(Hood.getInstance());

        hoodController = new ProfiledPIDController(HOOD_KP, HOOD_KI, HOOD_KD, new Constraints(10, 15));
        hoodController.setIntegratorRange(-HOOD_IZONE, HOOD_IZONE);
        feedforward = new SimpleMotorFeedforward(0, HOOD_KV, HOOD_KA);
        hoodController.setTolerance(0);

        referencePoints = new InterpolatedTreeMap();
        referencePoints.put(1.08, 10.0);
        referencePoints.put(1.18, 16.5);
        referencePoints.put(1.33, 15.5); //prac
        // referencePoints.put(1.4, 17.5);
        referencePoints.put(1.58, 18.5);
        referencePoints.put(1.89, 20.0);
        referencePoints.put(2.27, 23.0);
    }

    public void initialize() {
        hoodController.reset(Hood.getInstance().getHoodPos());
    }
    
    public void execute() {
        if(!Hood.isZeroed) return;
        // hoodController.setP(SmartDashboard.getNumber("hood P", HOOD_KP));
        // hoodController.setI(SmartDashboard.getNumber("hood I", HOOD_KI));
        // hoodController.setD(SmartDashboard.getNumber("hood D", HOOD_KD));
        // double izone = SmartDashboard.getNumber("hood izone", HOOD_IZONE);
        // hoodController.setIntegratorRange(-izone, izone);

        hoodPosition = referencePoints.get(Limelight.getDistance());
        if(OI.getInstance().getOperatorGamepad().getButtonBumperRightState())
            hoodPosition = 3;
        // hoodPosition = SmartDashboard.getNumber("desired hood pos", 1);
        double controlEffort = hoodController.calculate(Hood.getInstance().getHoodPos(), hoodPosition);
        double feedforwardAmount = feedforward.calculate(hoodController.getSetpoint().velocity) + HOOD_KG;
        double output = controlEffort + feedforwardAmount;
        output += Math.signum(output) * HOOD_KS;
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, output/Units.MAX_CONTROL_EFFORT);
        SmartDashboard.putNumber("hood pid goal", hoodController.getGoal().position);
        SmartDashboard.putNumber("hood pid setpoint", hoodController.getSetpoint().position);
        SmartDashboard.putNumber("hood pid error", hoodController.getPositionError());
        SmartDashboard.putNumber("hood pid control effort", controlEffort);
        SmartDashboard.putNumber("hood pid feedforward", feedforwardAmount);
    }

    public void end(boolean interrupted) {
        hoodPosition = 0;
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, 0);
    }
}
