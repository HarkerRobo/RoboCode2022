package frc.robot.commands.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.drivetrain.AlignWithLimelight;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.hood.HoodManual;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

public class MoveBallsToShooter extends IndefiniteCommand {
    
    public static final double SPEED = 0.3;
    public static final double LIMELIGHT_THRESHOLD = 5;
    public static final double HUB_RADIUS = 0.6096;
    private static Debouncer debouncer = new Debouncer(0.07, DebounceType.kFalling);
    
    public MoveBallsToShooter() {
        addRequirements(Indexer.getInstance());
    }
    
    public void execute() {
        ChassisSpeeds speed = Drivetrain.getInstance().getKinematics().toChassisSpeeds(
            Drivetrain.getInstance().getTopLeft().getState(),
            Drivetrain.getInstance().getTopRight().getState(), 
            Drivetrain.getInstance().getBottomLeft().getState(), 
                Drivetrain.getInstance().getBottomRight().getState());
        
        double translationMag = Math.sqrt(speed.vxMetersPerSecond * speed.vxMetersPerSecond + speed.vyMetersPerSecond * speed.vyMetersPerSecond);
        double limelightThreshold = Math.atan(HUB_RADIUS/Limelight.getDistance()) * 2 + 1;
        if(Math.abs(HoodManual.hoodController.getPositionError()) <= 0.15 && Math.abs(Limelight.getTx()) <= limelightThreshold &&
            Math.abs(translationMag) <= 0.2 && debouncer.calculate(Math.abs(Shooter.getInstance().getWheelRPS() - 
            Shooter.getInstance().getVelocitySystem().getLinearSystemLoop().getNextR(0)) <= 3)) {
            Indexer.getInstance().setPercentOutputBottom(SPEED);
            Indexer.getInstance().setPercentOutputTop(SPEED);
        }
        else {
            Indexer.getInstance().setPercentOutputBottom(0);
            Indexer.getInstance().setPercentOutputTop(0);
        }
    }
}