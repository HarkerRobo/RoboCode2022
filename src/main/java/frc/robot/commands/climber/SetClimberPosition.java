package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetClimberPosition extends CommandBase {
    private static final double MAX_ERROR = 500;
    private boolean limitSwitch;
    private double position;
    private double speed;

    public SetClimberPosition(double pos, double speed) {
        addRequirements(Climber.getInstance());
        this.position = pos;
        this.speed = Math.abs(speed);
        limitSwitch = Math.abs(pos) < 1;
    }

    public void execute() {
        if(!Climber.getInstance().isZeroed && !limitSwitch) return;
        double direction = Math.signum(position - Climber.getInstance().getPositionLeft());
        if(limitSwitch) direction = -1;
        Climber.getInstance().setClimberOutput(direction * Math.abs(speed));
        // if(limitSwitch)
        // {
        //     if(Climber.getInstance().getClimberLeft().isRevLimitSwitchClosed() == 1 || Climber.getInstance().getClimberLeft().isFwdLimitSwitchClosed() == 1)
        //     {
        //         double direction = Math.signum(position - Climber.getInstance().getPositionLeft());
        //         Climber.getInstance().setClimberOutputLeft(direction * Math.abs(speed));
        //     }
        //     else
        //     {
        //         Climber.getInstance().setClimberOutputLeft(ClimberManual.MAGNITUDE_IDLE);
        //     }
        //     if(Climber.getInstance().getClimberRight().isRevLimitSwitchClosed() == 1 || Climber.getInstance().getClimberRight().isFwdLimitSwitchClosed() == 1)
        //     {
        //         double direction = Math.signum(position - Climber.getInstance().getPositionRight());
        //         Climber.getInstance().setClimberOutputRight(direction * Math.abs(speed));
        //     }
        //     else
        //     {
        //         Climber.getInstance().setClimberOutputRight(ClimberManual.MAGNITUDE_IDLE);
        //     }
        // }
        // else
        // {
        //     if(Math.abs(position - Climber.getInstance().getPositionLeft()) < MAX_ERROR)
        //     {
        //         double direction = Math.signum(position - Climber.getInstance().getPositionLeft());
        //         Climber.getInstance().setClimberOutputLeft(direction * Math.abs(speed));
        //     }
        //     else
        //     {
        //         Climber.getInstance().setClimberOutputLeft(ClimberManual.MAGNITUDE_IDLE);
        //     }
        //     if(Math.abs(position - Climber.getInstance().getPositionRight()) < MAX_ERROR)
        //     {
        //         double direction = Math.signum(position - Climber.getInstance().getPositionRight());
        //         Climber.getInstance().setClimberOutputRight(direction * Math.abs(speed));
        //     }
        //     else
        //     {
        //         Climber.getInstance().setClimberOutputRight(ClimberManual.MAGNITUDE_IDLE);
        //     }
        // }
    }

    @Override
    public boolean isFinished() {
        if(limitSwitch)
            return Climber.getInstance().limitSwitchHit();//(Climber.getInstance().getClimberLeft().isRevLimitSwitchClosed() == 1 || Climber.getInstance().getClimberLeft().isFwdLimitSwitchClosed() == 1)
                    //&& (Climber.getInstance().getClimberRight().isRevLimitSwitchClosed() == 1 || Climber.getInstance().getClimberRight().isFwdLimitSwitchClosed() == 1);
        return (Math.abs(position - Climber.getInstance().getPositionLeft()) < MAX_ERROR || Math.abs(position - Climber.getInstance().getPositionRight()) < MAX_ERROR);
    }

    public void end(boolean interrupted) {
        System.out.println("down");
        if(limitSwitch && !interrupted)
        {
            Climber.getInstance().isZeroed = true;
            Climber.getInstance().getClimberLeft().setSelectedSensorPosition(0);
            Climber.getInstance().getClimberRight().setSelectedSensorPosition(0);
        }
        Climber.getInstance().setClimberOutput(ClimberManual.MAGNITUDE_IDLE);
    }
}