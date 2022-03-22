package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

public class AlignWithLimelight extends IndefiniteCommand {
    private static final double LIMELIGHT_THRESHOLD = 1.5;
    public static final double LIMELIGHT_KP = SwerveManual.LIMELIGHT_KP*1.2;
    public static final double LIMELIGHT_KD = SwerveManual.LIMELIGHT_KD;
    public static final double LIMELIGHT_IZONE = SwerveManual.LIMELIGHT_IZONE;
    public static final double LIMELIGHT_KI = SwerveManual.LIMELIGHT_KI*2;
    private ProfiledPIDController txController;
    private Timer minTime;

    public AlignWithLimelight() {
        addRequirements(Drivetrain.getInstance());
        txController = new ProfiledPIDController(LIMELIGHT_KP, LIMELIGHT_KI, LIMELIGHT_KD,  new Constraints(4, 4));
        txController.setIntegratorRange(-LIMELIGHT_IZONE, LIMELIGHT_IZONE);
        minTime = new Timer();

    }

    public void initialize() {
        minTime.reset();
        minTime.start();
        txController.reset(0);
    }

    public void execute() {
        Limelight.update();
        double angularVelocity = -txController.calculate(Limelight.getTx(), 0);
        ChassisSpeeds chassis = new ChassisSpeeds(0, 0, -angularVelocity);
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), false);
        SmartDashboard.putNumber("limelight align error", txController.getPositionError());
    }

    public boolean isFinished(){
        return minTime.hasElapsed(2) && Math.abs(Limelight.getTx()) < LIMELIGHT_THRESHOLD;
    }

    public void end(boolean isFinished){
        System.out.println("end");
        ChassisSpeeds chassis = new ChassisSpeeds(0,0,0);
        // System.out.println()
        // SwerveManual.pigeonAngle = Drivetrain.getInstance().getHeading();
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), true);
    }
}
