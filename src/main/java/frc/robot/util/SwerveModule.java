package frc.robot.util;

import frc.robot.RobotMap;
import frc.robot.Units;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.wrappers.HSFalcon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Specifies a rotation and translation motor, using PID to correct the
 * rotation motor and SimpleVelocitySystem to correct the translation motor.
 */
public class SwerveModule {
	public HSFalcon rotation;
	public HSFalcon translation;

	private static final double VOLTAGE_COMP = 10;

	private static final double ANGLE_MOTOR_CURRENT_CONTINUOUS = 25;
    private static final double ANGLE_MOTOR_CURRENT_PEAK = 40;
    private static final double ANGLE_MOTOR_CURRENT_PEAK_DUR = 0.1;

	private static final double DRIVE_MOTOR_CURRENT_CONTINUOUS = 35;
    private static final double DRIVE_MOTOR_CURRENT_PEAK = 60;
    private static final double DRIVE_MOTOR_CURRENT_PEAK_DUR = 0.1;
	
	private static final double ANGLE_P = 0.3;//0.23082;
	private static final double ANGLE_I = 0;
	private static final double ANGLE_D = 0;

	private static final double DRIVE_KS = -0.3;//-0.62769/2;
	private static final double DRIVE_KV = 2.2819;
	private static final double DRIVE_KA = 0.3621;

	private static final double MAX_ERROR = 1;  
    private static final double MODEL_STANDARD_DEVIATION = 0.5;
    private static final double ENCODER_STANDARD_DEVIATION = 0.035;
	

	private boolean ROTATION_INVERT;
	private boolean TRANSLATION_INVERT;

	private SimpleVelocitySystem translationLoop;
	private SwerveModuleState state;
	private CANCoder rotationEncoder;
	public SwerveModule(int rotationDriveId, int rotationEncoderID,
			int translationDriveId, boolean rotationInverted, boolean translationInverted) {
		ROTATION_INVERT = rotationInverted;
		TRANSLATION_INVERT = translationInverted;

		rotation = new HSFalcon(rotationDriveId, RobotMap.CANIVORE);
		translation = new HSFalcon(translationDriveId, RobotMap.CANIVORE);
		rotationEncoder = new CANCoder(rotationEncoderID, RobotMap.CANIVORE);
		rotationMotorInit();
		translationMotorInit();
		translationLoop = new SimpleVelocitySystem(DRIVE_KS, DRIVE_KV, DRIVE_KA, MAX_ERROR, Units.MAX_CONTROL_EFFORT, MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, RobotMap.LOOP_TIME);
		state = new SwerveModuleState();
	}

	private void rotationMotorInit() {
		rotation.configFactoryDefault();
		rotation.setInverted(ROTATION_INVERT);
		rotation.setNeutralMode(NeutralMode.Brake);
		rotation.configVoltageCompSaturation(VOLTAGE_COMP);
		rotation.configForwardSoftLimitEnable(false);
		rotation.configOpenloopRamp(0.1);
		rotation.configClosedloopRamp(0.1);
		rotation.config_kP(RobotMap.SLOT_INDEX, ANGLE_P);
		rotation.config_kI(RobotMap.SLOT_INDEX, ANGLE_I);
		rotation.config_kD(RobotMap.SLOT_INDEX, ANGLE_D);
		rotation.configVelocityMeasurementWindow(16);
		rotation.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
		rotation.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, ANGLE_MOTOR_CURRENT_CONTINUOUS, ANGLE_MOTOR_CURRENT_PEAK, ANGLE_MOTOR_CURRENT_PEAK_DUR));

		rotation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);

		rotation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.SLOT_INDEX);
	}

	private void translationMotorInit() {
		translation.configFactoryDefault();
		translation.setNeutralMode(NeutralMode.Brake);
		translation.setInverted(TRANSLATION_INVERT);

		translation.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, DRIVE_MOTOR_CURRENT_CONTINUOUS, DRIVE_MOTOR_CURRENT_PEAK, DRIVE_MOTOR_CURRENT_PEAK_DUR));
		translation.configVoltageCompSaturation(VOLTAGE_COMP);
		translation.configOpenloopRamp(0.3);
		translation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);
		translation.configVelocityMeasurementWindow(4);
		translation.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
		
		translation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.SLOT_INDEX);
	}

	public CANCoder getCanCoder() {
		return rotationEncoder;
	}

	public HSFalcon getTranslationMotor() {
		return translation;
	}

	public SimpleVelocitySystem getTranslationLoop() {
		return translationLoop;
	}

	public HSFalcon getRotationMotor() {
		return rotation;
	}

	public double getRotationAngle() {
		return rotation.getSelectedSensorPosition()*(Units.FALCON_ENCODER_TO_DEGREE) / Drivetrain.ROTATION_GEAR_RATIO;
	}

	public double getTranslationVelocity() {
		return translation.getSelectedSensorVelocity() / Units.FALCON_ENCODER_TICKS * 10 * Units.WHEEL_ROT_TO_METER / Drivetrain.TRANSLATION_GEAR_RATIO;
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
		double speed = state.speedMetersPerSecond;
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
			speed *= -1;
		}

		else if(angle-currentAngle<-90){
			angle += 180;
			speed *= -1;
		}
		rotation.set(ControlMode.Position, angle / Units.FALCON_ENCODER_TO_DEGREE * Drivetrain.ROTATION_GEAR_RATIO);
		if(isPercentOutput)
			speed /= Drivetrain.MAX_DRIVE_VEL;
		setDriveOutput(speed, isPercentOutput);
	}

	public void setDriveOutput(double output, boolean isPercentOutput) {
		if(Math.abs(output) < Drivetrain.MIN_OUTPUT*5){
			output = 0;
		}
        if(!isPercentOutput) {
            translationLoop.set(output);
			translationLoop.update(getTranslationVelocity());
		}
        translation.set(TalonFXControlMode.PercentOutput, (isPercentOutput) ? output: translationLoop.getOutput());
	}
}