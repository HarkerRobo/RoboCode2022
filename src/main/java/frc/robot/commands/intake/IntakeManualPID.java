package frc.robot.commands.intake;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeManualPID extends IndefiniteCommand {
    private static final double MAX_VELOCITY = 15;
    private static final double AGITATOR_MAX_SPEED = 1;
    private static final double LINEAR_MAX_SPEED = 0.54;
    private long commandTime;
    private double magnitude;
    private boolean reverse;

    public IntakeManualPID(double magnitude, boolean reverse) {
        addRequirements(Intake.getInstance());
        this.magnitude = magnitude;
        this.reverse = reverse;
    }

    @Override
    public void initialize() {
        commandTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        Intake.getInstance().setVelocity(magnitude*MAX_VELOCITY);

        if(reverse)

            Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_OPEN);

        else
            Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_CLOSED);


        boolean blocked = Indexer.getInstance().shooterSensorBlocked() && Indexer.getInstance().linearSensorBlocked();
        SmartDashboard.putBoolean("Blocked", blocked);
        SmartDashboard.putBoolean("Shooter Blocked", Indexer.getInstance().shooterSensorBlocked());
        SmartDashboard.putBoolean("Linear Blocked", Indexer.getInstance().linearSensorBlocked());


        if (!blocked || reverse) {
            Indexer.getInstance().setLinearPercentOutput(reverse ? -LINEAR_MAX_SPEED : LINEAR_MAX_SPEED);
        } else{
            Indexer.getInstance().setLinearPercentOutput(0);
        }
        if (commandTime % 1000 < 500) {
            Indexer.getInstance().setAgitatorPercentOutput(AGITATOR_MAX_SPEED);
        } else {
            Indexer.getInstance().setAgitatorPercentOutput(-AGITATOR_MAX_SPEED);
        }


        commandTime = System.currentTimeMillis();

    }

    @Override
    public void end(boolean a) {
        Intake.getInstance().setVelocity(0);
        Indexer.getInstance().setLinearPercentOutput(0);

        Indexer.getInstance().setAgitatorPercentOutput(0);

        Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_CLOSED);
    }
}
