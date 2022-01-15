package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Intake extends SubsystemBase {
    private static Intake intake;
    private HSFalcon motor;
    private DoubleSolenoid doubleSolenoid;
    

    private Intake() {
        motor = new HSFalcon(RobotMap.INTAKE_ID);
        doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_FORWARD, RobotMap.INTAKE_BACKWARD);

    }
    

    public void init(){
        
    }
    
    public HSFalcon getMotor(){
        return motor;
    }
    
    public void setVelocity(double vel){
        motor.set(ControlMode.Velocity, vel);
    }

    public void toggle() {
        if (doubleSolenoid.get() == DoubleSolenoid.Value.kForward){
            doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        else{
            doubleSolenoid.set(DoubleSolenoid.Value.kForward);
        }
    }

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }

}