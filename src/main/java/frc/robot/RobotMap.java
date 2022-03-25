package frc.robot;

/**
 * Defines robot-wide constants
 */
public class RobotMap {
    public static final boolean IS_COMP = true;

    public static final int LOOP_INDEX = 0;

    public static final int SLOT_INDEX = 0;
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final String CANIVORE = (IS_COMP) ? "BINGCHILLING" : "RIO";

    public static final int[] TRANSLATION_IDS = {13, 4, 2, 6}; // TL, TR, BL, BR
    public static final int[] ROTATION_IDS = {1, 5, 3, 7}; // TL, TR, BL, BR
    public static final int[] ROTATION_CANCODER_IDS = {0, 2, 1, 3}; //TL, TR, BL, BR

    public static final int PIGEON_ID = 0;

    public static final int INTAKE_ID = 8;
    public static final int INTAKE_FORWARD = RobotMap.IS_COMP ? 7 : 0;
    public static final int INTAKE_BACKWARD = RobotMap.IS_COMP ? 3 : 7;

    public static final int SHOOTER_MASTER = 11;
    public static final int SHOOTER_FOLLOWER = 12;
    public static final int SHOOTER_ENCODER_A = 8;
    public static final int SHOOTER_ENCODER_B = 9;

    public static final int HOOD = 14;
    public static final int HOOD_ENCODER = 7;

    public static final int CLIMBER_RIGHT = 15;
    public static final int CLIMBER_LEFT = 16;
    public static final int CLIMBER_FORWARD =  RobotMap.IS_COMP ? 2 : 1; //change
    public static final int CLIMBER_BACKWARD =  RobotMap.IS_COMP ? 6: 6; //right
    public static final int CLIMBER_RIGHT_LIM_SWITCH = 4;
    public static final int CLIMBER_LEFT_LIM_SWITCH = 5;

    public static final int INDEXER_TOP = 10;
    public static final int INDEXER_BOTTOM = 9;
    public static final int INDEXER_TOP_SENSOR = 7;
    public static final int INDEXER_BOTTOM_SENSOR = 6;

    public static final double LOOP_TIME = 0.02; // 20ms
}