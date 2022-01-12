package frc.robot.commands.shooter;
import harkerrobolib.commands.IndefiniteCommand;
import frc.robot.subsystems.Shooter;

public class ShooterAutonControl extends IndefiniteCommand {
    private double velocity;

    public ShooterAutonControl(double velocity){
        this.velocity = velocity;
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void execute(){
        Shooter.getInstance().setVelocity(velocity);
    }

    @Override
    public void end(boolean a){
        Shooter.getInstance().setVelocity(0);
    }
}
