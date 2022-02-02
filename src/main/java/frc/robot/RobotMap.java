package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RobotMap {
    public static final int LOOP_INDEX = 0;

    public static final int SLOT_INDEX = 0;
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;

    public static final int[] TRANSLATION_IDS = {0, 2, 4, 6}; // TL, TR, BL, BR
    public static final int[] ROTATION_IDS = {1, 3, 5, 7}; // TL, TR, BL, BR
    public static final int[] ROTATION_CANCODER_IDS = {0, 1, 2, 3}; //TL, TR, BL, BR

    public static final int PIGEON_ID = 0;

    public static final int INTAKE_ID = 8;
    public static final int INTAKE_FORWARD = 0;
    public static final int INTAKE_BACKWARD = 0;

    public static final int SHOOTER_MASTER = 10;
    public static final int SHOOTER_FOLLOWER = 11;  // change

    public static final double LOOP_TIME = 0.02; // 20ms

    public static final boolean IS_COMP = false;
    public static final double PIGEON_CONSTANT = 63.9886;
}