package frc.robot.commands.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
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
    private double limelightThreshold;
    private static Debouncer debouncer = new Debouncer(0.07, DebounceType.kFalling);
    private boolean override;
    private double shootspeed;
    private boolean isHood, isLimelight, isTx, isTranslation, isRotation, isShooter;
    
    public MoveBallsToShooter(boolean override) {
        addRequirements(Indexer.getInstance());
        this.override = override;
        this.shootspeed = SPEED;
    }

    public MoveBallsToShooter(double autospeed) {
        addRequirements(Indexer.getInstance());
        this.override = true;
        this.shootspeed = autospeed;
    }
    
    
    public void execute() {
        SmartDashboard.putData(this);
        ChassisSpeeds speed = Drivetrain.getInstance().getKinematics().toChassisSpeeds(
            Drivetrain.getInstance().getTopLeft().getState(),
            Drivetrain.getInstance().getTopRight().getState(), 
            Drivetrain.getInstance().getBottomLeft().getState(), 
                Drivetrain.getInstance().getBottomRight().getState());
        
        double translationMag = Math.sqrt(speed.vxMetersPerSecond * speed.vxMetersPerSecond + speed.vyMetersPerSecond * speed.vyMetersPerSecond);
        limelightThreshold = Math.toDegrees(Math.atan(HUB_RADIUS/(Limelight.getDistance() + HUB_RADIUS))) *0.9;

        isHood = Math.abs(HoodManual.hoodController.getPositionError()) <= 0.5;
        isLimelight = Limelight.isTargetVisible();
        isTx = Math.abs(Limelight.getTx()) <= limelightThreshold;
        isTranslation = Math.abs(translationMag) <= 0.2;
        isRotation = Math.abs(speed.omegaRadiansPerSecond) <= 0.3;
        isShooter = debouncer.calculate(Math.abs(Shooter.getInstance().getWheelRPS() - 
                            Shooter.getInstance().getVelocitySystem().getLinearSystemLoop().getNextR(0)) <= 3);
        if((((isHood && isLimelight && isTx && isTranslation && isRotation) || override) && isShooter) || OI.getInstance().getOperatorGamepad().getRightTrigger() > 0.5){
            Indexer.getInstance().setPercentOutputBottom(shootspeed);
            Indexer.getInstance().setPercentOutputTop(shootspeed);
        }
        else {
            Indexer.getInstance().setPercentOutputBottom(0);
            Indexer.getInstance().setPercentOutputTop(0);
        }
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Move Balls to Shooter");
        builder.addDoubleProperty("Limelight Threshold", () -> limelightThreshold, null);
        builder.addBooleanProperty("Hood at Position", () -> isHood, null);
        builder.addBooleanProperty("Limelight is Visible", () -> isLimelight, null);
        builder.addBooleanProperty("Tx within Threshold", () -> isTx, null);
        builder.addBooleanProperty("Robot not Translating", () -> isTranslation, null);
        builder.addBooleanProperty("Robot not Rotating", () -> isRotation, null);
        builder.addBooleanProperty("Shooter at Velocity", () -> isShooter, null);
    }
}