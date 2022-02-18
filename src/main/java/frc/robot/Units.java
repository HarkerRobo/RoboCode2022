package frc.robot;

import frc.robot.subsystems.Drivetrain;

public class Units {
    public static final int FALCON_ENCODER_TICKS = 2048;
    public static final int MAG_CODER_ENCODER_TICKS = 4096;

    public static final double FALCON_ENCODER_TO_DEGREE = 360.0/FALCON_ENCODER_TICKS;

    public static final double IN_TO_METER = 0.0254;

    public static final double WHEEL_ROT_TO_METER = Drivetrain.WHEEL_DIAMETER * Math.PI * IN_TO_METER;
    public static final double MAX_CONTROL_EFFORT = 10;
}
