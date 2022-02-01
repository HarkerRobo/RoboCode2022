package frc.robot.util;

import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.wrappers.HSFalcon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import harkerrobolib.util.Conversions.SpeedUnit;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import harkerrobolib.util.Conversions;

/**
 * Specifies a rotation and translation motor, using PID to correct the
 * rotation motor and SimpleVelocitySystem to correct the translation motor.
 */
public class SwerveModule {
	public HSFalcon rotation;
	public HSFalcon translation;

	private static final double VOLTAGE_COMP = 10;

	private static final double DRIVE_CURRENT_CONTINUOUS = 40;
    private static final double DRIVE_CURRENT_PEAK = 60;
    private static final double DRIVE_CURRENT_PEAK_DUR = 0.4;

    private static final double ANGLE_CURRENT_CONTINUOUS = 15;
    private static final double ANGLE_CURRENT_PEAK = 20;
    private static final double ANGLE_CURRENT_PEAK_DUR = 0.02;

	// private static final double TRANSLATION_P = 0.5;
	// private static final double TRANSLATION_I = 0.0;
	// private static final double TRANSLATION_D = 5;
	// private static final double TRANSLATION_F = 0.034;
	
	private static final double ANGLE_P = 1.1;
	private static final double ANGLE_I = 0;
	private static final double ANGLE_D = 11;
	private static final int ENCODER_TICKS = 2048;

	private static final double DRIVE_KS = 0.578;
	private static final double DRIVE_KV = 2.0473;
	private static final double DRIVE_KA = 0.13018;

	private static final double MAX_CONTROL_EFFORT = 10; // volts 
    private static final double MODEL_STANDARD_DEVIATION = 2;
    private static final double ENCODER_STANDARD_DEVIATION = 0.08;

	private boolean ROTATION_SENSOR_PHASE;
	private boolean TRANSLATION_SENSOR_PHASE;
	
	private boolean ROTATION_INVERT;
	private boolean TRANSLATION_INVERT;

	private SimpleVelocitySystem translationLoop;
	private SwerveModuleState state;
	private CANCoder rotationEncoder;

	public SwerveModule(boolean rotationSensorPhase, boolean translationSensorPhase, int rotationDriveId, int rotationEncoderID,
			int translationDriveId, boolean rotationInverted, boolean translationInverted) {
		ROTATION_SENSOR_PHASE = rotationSensorPhase;
		TRANSLATION_SENSOR_PHASE = translationSensorPhase;

		ROTATION_INVERT = rotationInverted;
		TRANSLATION_INVERT = translationInverted;

		rotation = new HSFalcon(rotationDriveId);
		translation = new HSFalcon(translationDriveId);
		rotationEncoder = new CANCoder(rotationEncoderID);
		rotationMotorInit();
		translationMotorInit();
		translationLoop = new SimpleVelocitySystem(DRIVE_KS, DRIVE_KV, DRIVE_KA, MAX_CONTROL_EFFORT, MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, RobotMap.LOOP_TIME);
		state = new SwerveModuleState();
	}

	private void rotationMotorInit() {
		rotation.configFactoryDefault();
		rotation.setInverted(ROTATION_INVERT);
		rotation.setSensorPhase(ROTATION_SENSOR_PHASE);
		rotation.setNeutralMode(NeutralMode.Brake);
		rotation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, ANGLE_CURRENT_CONTINUOUS, ANGLE_CURRENT_PEAK, ANGLE_CURRENT_PEAK_DUR));
		rotation.configVoltageCompSaturation(VOLTAGE_COMP);
		rotation.configForwardSoftLimitEnable(false);

		rotation.config_kP(RobotMap.SLOT_INDEX, ANGLE_P);
		rotation.config_kI(RobotMap.SLOT_INDEX, ANGLE_I);
		rotation.config_kD(RobotMap.SLOT_INDEX, ANGLE_D);
		rotation.configVelocityMeasurementWindow(16);
		rotation.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);

		rotation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);

		rotation.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.SLOT_INDEX);
	}

	private void translationMotorInit() {
		translation.configFactoryDefault();
		translation.setSensorPhase(TRANSLATION_SENSOR_PHASE);
		translation.setNeutralMode(NeutralMode.Brake);
		translation.setInverted(TRANSLATION_INVERT);

		translation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DRIVE_CURRENT_CONTINUOUS, DRIVE_CURRENT_PEAK, DRIVE_CURRENT_PEAK_DUR));

		translation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);
		translation.configVelocityMeasurementWindow(1);
		translation.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
		
		translation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.SLOT_INDEX);
	}

	public CANCoder getCanCoder() {
		return rotationEncoder;
	}

	public HSFalcon getTranslationMotor() {
		return translation;
	}

	public HSFalcon getRotationMotor() {
		return rotation;
	}

	public double getRotationAngle() {
		return rotation.getSelectedSensorPosition()*(360.0/ENCODER_TICKS) / Drivetrain.ROTATION_GEAR_RATIO;
	}

	private double getTranslationVelocity() {
		return Conversions.convertSpeed(SpeedUnit.ENCODER_UNITS, translation.getSelectedSensorVelocity(), SpeedUnit.FEET_PER_SECOND, Drivetrain.WHEEL_DIAMETER, ENCODER_TICKS) / Drivetrain.FEET_TO_METER / Drivetrain.TRANSLATION_GEAR_RATIO;
	}
	
	public SwerveModuleState getState() {
		state.speedMetersPerSecond = getTranslationVelocity();
		state.angle = Rotation2d.fromDegrees(getRotationAngle());
		return state;
	}

	public void setSwerveManual(SwerveModuleState state, boolean isPercentOutput){
		//state = optimize(state, getRotationAngle());
		double angle = state.angle.getDegrees();
		double currentAngle = getRotationAngle();
		while(angle-currentAngle>180){
			angle -= 360;
			//delta = desiredState.angle.getDegrees() - currentAngle;
		}

		while(angle-currentAngle<-180){
			angle += 360;
			//delta = desiredState.angle.minus(currentAngle);
		}

		if(angle-currentAngle>90){
			angle -= 180;
		}

		else if(angle-currentAngle<-90){
			angle += 180;
		}
		rotation.set(ControlMode.Position, Drivetrain.ROTATION_GEAR_RATIO * angle * (ENCODER_TICKS / 360));
		double speed = state.speedMetersPerSecond;
		if(isPercentOutput)
			speed /= Drivetrain.MAX_DRIVE_VEL;
		setDriveOutput(speed, isPercentOutput);
	}

	public void setDriveOutput(double output, boolean isPercentOutput) {
        if(!isPercentOutput) {
            translationLoop.set(output);
			translationLoop.update(getTranslationVelocity());
		}
        translation.set(TalonFXControlMode.PercentOutput, (isPercentOutput) ? output: translationLoop.getOutput());
	}
}