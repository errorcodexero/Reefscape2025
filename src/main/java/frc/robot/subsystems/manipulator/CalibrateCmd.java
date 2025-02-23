package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;

public class CalibrateCmd extends Command {
    private ManipulatorSubsystem m_ ;
    private boolean elev_calibrated_ ;
    private int count_ ;

    public CalibrateCmd(ManipulatorSubsystem manipulatorSubsystem) {
        addRequirements(manipulatorSubsystem);
        m_ = manipulatorSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_.enableSoftLimits(false);
        m_.setElevatorVoltage(ManipulatorConstants.Elevator.kCalibrateVoltage);
        elev_calibrated_ = false ;
        count_ = 0 ;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_.getElevatorVelocity().abs(MetersPerSecond) < 0.01) {
            count_++ ;
            if (count_ == ManipulatorConstants.Elevator.kCalibrateLoops) {
                elev_calibrated_ = true ;
                m_.setElevatorVoltage(Volts.of(0.0)) ;
                m_.resetElevator() ;
                m_.enableSoftLimits(true);
            }
        }
        else {
            count_ = 0 ;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return elev_calibrated_ ;
    }
}
