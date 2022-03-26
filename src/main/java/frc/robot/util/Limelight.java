package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Wraps Limelight functionality into a class.
 * 
 * @author Finn Frankis
 * @author Chirag Kaushik
 * 
 * @since 01/20/20
 */
public class Limelight {
    public static final String LIMELIGHT_TABLE_KEY = "limelight";
    public static final NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY);

    public static final String TV_KEY = "tv";
    public static final String TX_KEY = "tx";
    public static final String TY_KEY = "ty";
    public static final String TA_KEY = "ta";
    public static final String TS_KEY = "ts";
    public static final String TL_KEY = "tl";
    public static final String TSHORT_KEY = "tshort";
    public static final String TLONG_KEY = "tlong";
    public static final String THOR_KEY = "thor";
    public static final String TVERT_KEY = "tvert";
    public static final String MODE_KEY = "camMode";
    public static final String SNAP_KEY = "snapshot";
    public static final String CORNERX_KEY = "tcornx";
    public static final String CORNERY_KEY = "tcorny";
    public static final String CAMTRAN_KEY = "camtran";
    public static final String PIPELINE_KEY = "pipeline";
    public static final String LED_MODE = "ledMode";

    public static final int LED_PIPELINE = 0;
    public static final int LED_OFF = 1;
    public static final int LED_BLINK = 2;
    public static final int LED_ON = 3;

    public static final int VISION_MODE = 0;
    public static final int DRIVER_MODE = 1;

    public static final int NO_SNAPSHOT = 0;
    public static final int SNAPSHOT = 1;

    public static final int LIMELIGHT_ANGLE = 43;
    
    private static double[] nullArr;    
    private static LinearFilter txFilter = LinearFilter.movingAverage(5);
    private static LinearFilter tyFilter = LinearFilter.movingAverage(5);
    private static double currentTx;
    private static double currentTy;
    public static final double LIMELIGHT_HEIGHT = 0.94;
    public static final double TARGET_HEIGHT = 2.64;


    /**
     * Setup Limelight with default settings
     */
    private Limelight() {
        table.getEntry(MODE_KEY).setNumber(VISION_MODE);
        table.getEntry(SNAP_KEY).setNumber(NO_SNAPSHOT);
    }

    /**
     * Determines whether a target has been latched onto.
     * 
     * @return true if a target is visible; otherwise, false
     */
    public static boolean isTargetVisible() {
        return Math.abs(table.getEntry(TV_KEY).getDouble(0.0) - 1.0) < 1e-5;
    }

    /**
     * Determines the horizontal angular distance from the crosshair to the center
     * of the bounding box representing the target.
     * 
     * @return the horizontal angular distance to the target, in degrees
     */
    public static double getTx() {
        return currentTx;
    }

    public static void update() {
        currentTx = txFilter.calculate(table.getEntry(TX_KEY).getDouble(0.0));
        currentTy = tyFilter.calculate(table.getEntry(TY_KEY).getDouble(0.0));
    }

    /**
     * Toggles the limelight between driver and vision mode.
     */
    public static void toggleCamMode() {
        if (table.getEntry(MODE_KEY).getDouble(0) == VISION_MODE)
            table.getEntry(MODE_KEY).setNumber(DRIVER_MODE);
        else
            table.getEntry(MODE_KEY).setNumber(VISION_MODE);
    }

    /**
     * Toggles the limelight LEDs on or off.
     */
    public static void toggleLEDs() {
        if(table.getEntry(LED_MODE).getNumber(0).intValue() == LED_ON) 
            table.getEntry(LED_MODE).setNumber(LED_OFF);
        else
            table.getEntry(LED_MODE).setNumber(LED_ON);
    }

    /**
     * Sets the LEDS to be on or off
     */
    public static void setLEDS(boolean on) {
        table.getEntry(LED_MODE).setNumber(on ? LED_ON : LED_OFF);
    }

    public static void setCamModeDriver() {
        table.getEntry(MODE_KEY).setNumber(DRIVER_MODE);
    }

    public static void setCamModeVision() {
        table.getEntry(MODE_KEY).setNumber(VISION_MODE);
    }

    /**
     * Determines the vertical angular distance from the crosshair to the center of
     * the bounding box representing the target.
     * 
     * @return the vertical, angular distance to the target, in degrees
     */
    public static double getTy() {
        return currentTy;
    }

    /**
     * Determines the angular skew of the bounding box representing the target.
     * 
     * @return the angular skew, in degrees [-90, 0]
     */
    public static double getTs() {
        return table.getEntry(TS_KEY).getDouble(0.0);
    }

    /**
     * Determines the area of the target.
     * 
     * @return the area, as a percent of the total screen
     */
    public static double getTa() {
        return table.getEntry(TA_KEY).getDouble(0.0);
    }

    /**
     * Determines the latency of the camera feed, or a measure of how long the feed
     * takes to process.
     * 
     * @return the latency, in milliseconds
     */
    public static double getTl() {
        return table.getEntry(TL_KEY).getDouble(0.0);
    }

    /**
     * Determines the sidelength of the shortest side of the fitted bounding box.
     * The fitted bounding box is a rectangular convex hull surrounding the selected
     * pixels, while the rough bounding box is a rectangle around the fitted
     * bounding box, put is not rotated
     * 
     * @return the shortest sidelength, in pixels
     */
    public static double getTshort() {
        return table.getEntry(TSHORT_KEY).getDouble(0.0);
    }

    /**
     * Determines the sidelength of the longest side of the fitted bounding box. The
     * fitted bounding box is a rectangular convex hull surrounding the selected
     * pixels, while the rough bounding box is a rectangle around the fitted
     * bounding box, put is not rotated
     * 
     * @return the longest sidelength, in pixels
     */
    public static double getTlong() {
        return table.getEntry(TLONG_KEY).getDouble(0.0);
    }

    /**
     * Determines the horizontal sidelength of the rough bounding box. The fitted
     * bounding box is a rectangular convex hull surrounding the selected pixels,
     * while the rough bounding box is a rectangle around the fitted bounding box,
     * put is not rotated
     * 
     * @return the horizontal sidelength, in pixels [0, 320]
     */
    public static double getThor() {
        return table.getEntry(THOR_KEY).getDouble(0.0);
    }

    /**
     * Determines the vertical sidelength of the rough bounding box. The fitted
     * bounding box is a rectangular convex hull surrounding the selected pixels,
     * while the rough bounding box is a rectangle around the fitted bounding box,
     * put is not rotated
     * 
     * @return the vertical sidelength, in pixels [0, 240]
     */
    public static double getTvert() {
        return table.getEntry(TVERT_KEY).getDouble(0.0);
    }

    /**
     * Returns the values of the Limelight's compute 3D localization calculations.
     * 
     * @return The compute 3D values, in the specified order (x,y,z,pitch,yaw,roll).
     *         These values place the target as the origin and represent the
     *         camera's orientation relative to the target.
     */
    public static double[] getCamtranData() {
        return table.getEntry(CAMTRAN_KEY).getDoubleArray(nullArr);
    }

    /**
     * Returns horizontal distance from the target
     * 
     * @return The camera's horizontal distance from the target.
     */
    public static double getCamtranX() {
        return getCamtranData()[0];
    }

    /**
     * Returns height difference between the the target
     * 
     * @return The camera's height relative to the target's height (automatically
     *         0).
     */
    public static double getCamtranY() {
        return getCamtranData()[1];
    }

    /**
     * Returns vertical distance from the target
     * 
     * @return The camera's vertical distance from the target.
     */
    public static double getCamtranZ() {
        return getCamtranData()[2];
    }

    /**
     * Returns camera's pitch (forward/backward tilt) relative to the target
     * 
     * @return The camera's pitch relative to the target (automatically 0).
     */
    public static double getCamtranPitch() {
        return getCamtranData()[3];
    }

    /**
     * Returns camera's yaw (left/right rotation) relative to the target
     * 
     * @return The camera's pitch relative to the target (automatically 0).
     */
    public static double getCamtranYaw() {
        return getCamtranData()[4];
    }

    /**
     * Returns camera's yaw (left/right tilt) relative to the target
     * 
     * @return The camera's pitch relative to the target (automatically 0).
     */
    public static double getCamtranRoll() {
        return getCamtranData()[5];
    }

    public static double getRawContourTx(int contourId) {
        return table.getEntry(TX_KEY + contourId).getDouble(0.0);
    }

    public static double getRawContourTy(int contourId) {
        return table.getEntry(TY_KEY + contourId).getDouble(0.0);
    }

    public static double getRawContourTa(int contourId) {
        return table.getEntry(TA_KEY + contourId).getDouble(0.0);
    }

    public static double getRawContourTs(int contourId) {
        return table.getEntry(TS_KEY + contourId).getDouble(0.0);
    }

    public static double[] getCornersX() {
        return table.getEntry(CORNERX_KEY).getDoubleArray(nullArr);
    }

    public static double[] getCornersY() {
        return table.getEntry(CORNERY_KEY).getDoubleArray(nullArr);
    }

    public static double getLeftArea() {
        return getRawContourTx(0) < getRawContourTx(1) ? getRawContourTa(0) : getRawContourTa(1);
    }

    public static double getRightArea() {
        return getRawContourTx(0) > getRawContourTx(1) ? getRawContourTa(0) : getRawContourTa(1);
    }

    public static void setPipeline(int pipeline) {
        table.getEntry(PIPELINE_KEY).setNumber(pipeline);
    }

    public static double getDistance() {
        return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / (Math.tan(Math.toRadians(getTy() + LIMELIGHT_ANGLE)) * Math.cos(Math.toRadians(getTx())));
    }
}