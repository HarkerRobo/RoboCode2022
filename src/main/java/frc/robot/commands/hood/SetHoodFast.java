
package frc.robot.commands.hood;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.Units;
import frc.robot.subsystems.Hood;
import frc.robot.util.InterpolatedTreeMap;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

public class SetHoodFast extends IndefiniteCommand{

    public static final double HOOD_KP = 0.1; 
    public static final double HOOD_KI = 0.01;//0.004;
    public static final double HOOD_KD = 0;//0.0020817; 
    public static double hoodIZone = 0.2; 
    public static final double HOOD_KS = RobotMap.IS_COMP ? 0.6 : 0.7; 

    public static final double HOOD_KV = 0.055112; 
    public static final double HOOD_KA = 0.0006622;
    public static final double HOOD_KG = 0.087132;
    
    private double hoodPosition;
    private double controlEffort;
    private double feedforwardAmount;
    private double output;
    public static boolean downMode = false;

    public static ProfiledPIDController hoodController = new ProfiledPIDController(HOOD_KP, HOOD_KI, HOOD_KD, new Constraints(40, 60));
    private InterpolatedTreeMap referencePoints;
    private SimpleMotorFeedforward feedforward;
    private double pos;
    public SetHoodFast(double pos){
        addRequirements(Hood.getInstance());
        this.pos = pos;
        hoodController.setIntegratorRange(-hoodIZone, hoodIZone);
        feedforward = new SimpleMotorFeedforward(0, HOOD_KV, HOOD_KA);
        hoodController.setTolerance(0.4);

    }

    public void initialize() {
        hoodController.reset(Hood.getInstance().getHoodPos());
    }
    
    public void execute() {
        SmartDashboard.putData(this);
        if(!Hood.isZeroed) return;
        hoodController.setIntegratorRange(-hoodIZone, hoodIZone);

        hoodPosition = pos;
        calculateMotorOutput();
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, output/Units.MAX_CONTROL_EFFORT);
    }

    public void calculateMotorOutput()
    {
        controlEffort = hoodController.calculate(Hood.getInstance().getHoodPos(), hoodPosition);
        feedforwardAmount = feedforward.calculate(hoodController.getSetpoint().velocity) + HOOD_KG;
        output = controlEffort + feedforwardAmount;
        output += Math.signum(output) * HOOD_KS;
    }

    @Override
    public boolean isFinished() {
        return hoodController.atGoal();
    }

    public void end(boolean interrupted) {
        System.out.println("ashiwn");
        hoodPosition = 0;
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, 0);
    }
}