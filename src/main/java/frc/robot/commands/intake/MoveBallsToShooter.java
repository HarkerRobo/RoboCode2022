package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;

public class MoveBallsToShooter extends IndefiniteCommand{
    private static final double AGITATOR_MAX_SPEED = 0.8;
    private static final double LINEAR_MAX_SPEED = 0.7;
    private static final long MIN_DELAY = 5000;
    private static final long OUT_DELAY = 100;
    private static final long PID_DELAY = 200;
    private long commandTime;
    private long pidTime=-1;

    public MoveBallsToShooter(){
        addRequirements(Indexer.getInstance());
        addRequirements(Intake.getInstance());
    }
    
    @Override
    public void initialize() {
        commandTime = System.currentTimeMillis();
        
    }

    @Override
    public void execute(){
        long currentTime = System.currentTimeMillis();
        

        if(currentTime - commandTime < OUT_DELAY) {
            Indexer.getInstance().setLinearPercentOutput(-LINEAR_MAX_SPEED);
            Indexer.getInstance().setAgitatorPercentOutput(AGITATOR_MAX_SPEED);
        }
        
        else if(!Shooter.isPercentOutput) {
                if(pidTime==-1){
                    pidTime = System.currentTimeMillis()
                ;}
                if(currentTime - pidTime > PID_DELAY) {
                    Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_OPEN);
                    Indexer.getInstance().setLinearPercentOutput(LINEAR_MAX_SPEED);


                    if (currentTime % 1000 < 500) {
                        Indexer.getInstance().setAgitatorPercentOutput(AGITATOR_MAX_SPEED);
                    } else {
                        Indexer.getInstance().setAgitatorPercentOutput(-AGITATOR_MAX_SPEED);
                    }
                }                
            }
        
        else {
            Indexer.getInstance().setLinearPercentOutput(0);
            Indexer.getInstance().setAgitatorPercentOutput(0);
        }
    }

    @Override
    public void end(boolean a){
        Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_CLOSED);
        Indexer.getInstance().setLinearPercentOutput(0); 
        Indexer.getInstance().setAgitatorPercentOutput(0);

    }
}
