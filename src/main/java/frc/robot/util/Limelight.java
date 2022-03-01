package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public class Limelight {

    public static double getTx() {
        Pose2d robotpose = Robot.field.getRobotPose();
        Translation2d robotvector = robotpose.getTranslation().plus(new Translation2d(3, robotpose.getRotation()));
        Translation2d robotToTargetVector =  new Translation2d(8.23, 4.11).minus(robotpose.getTranslation());
        double angle = Math.atan2(robotToTargetVector.getY(), robotToTargetVector.getX()) - Math.atan2(robotvector.getY(), robotvector.getX());
        if(angle < -Math.PI) angle += Math.PI;
        if(angle > Math.PI) angle -= Math.PI;
        angle = Math.toDegrees(angle);
        if(Math.abs(angle) > 20) return 0; // simulate cone of vision
        return angle;
    }
}
