package frc.robot.commands.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
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
    private boolean override;
    
    public MoveBallsToShooter(boolean override) {
        addRequirements(Indexer.getInstance());
        this.override = override;
    }
    
    public void execute() {
        ChassisSpeeds speed = Drivetrain.getInstance().getKinematics().toChassisSpeeds(
            Drivetrain.getInstance().getTopLeft().getState(),
            Drivetrain.getInstance().getTopRight().getState(), 
            Drivetrain.getInstance().getBottomLeft().getState(), 
                Drivetrain.getInstance().getBottomRight().getState());
        
        double translationMag = Math.sqrt(speed.vxMetersPerSecond * speed.vxMetersPerSecond + speed.vyMetersPerSecond * speed.vyMetersPerSecond);
        double limelightThreshold = Math.toDegrees(Math.atan(HUB_RADIUS/(Limelight.getDistance() + HUB_RADIUS))) + 1;

        boolean isHood = Math.abs(HoodManual.hoodController.getPositionError()) <= 0.5;
        boolean isLimelight = Limelight.isTargetVisible();
        boolean isTx = Math.abs(Limelight.getTx()) <= limelightThreshold;
        boolean isTranslation = Math.abs(translationMag) <= 0.2;
        boolean isRotation = Math.abs(speed.omegaRadiansPerSecond) <= 0.2;
        boolean isShooter = debouncer.calculate(Math.abs(Shooter.getInstance().getWheelRPS() - 
                            Shooter.getInstance().getVelocitySystem().getLinearSystemLoop().getNextR(0)) <= 3);
        SmartDashboard.putNumber("autoshot ll thresh", limelightThreshold);
        SmartDashboard.putBoolean("autoshot isHood", isHood);
        SmartDashboard.putBoolean("autoshot isLimelight", isLimelight);
        SmartDashboard.putBoolean("autoshot isTx", isTx);
        SmartDashboard.putBoolean("autoshot isTranslation", isTranslation);
        SmartDashboard.putBoolean("autoshot isRotation", isRotation);
        SmartDashboard.putBoolean("autoshot isShooter", isShooter);
        if((isHood && isLimelight && isTx && isTranslation && isRotation&& isShooter) || override || OI.getInstance().getOperatorGamepad().getRightTrigger() > 0.5){
            Indexer.getInstance().setPercentOutputBottom(SPEED);
            Indexer.getInstance().setPercentOutputTop(SPEED);
        }
        else {
            Indexer.getInstance().setPercentOutputBottom(0);
            Indexer.getInstance().setPercentOutputTop(0);
        }
    }
}