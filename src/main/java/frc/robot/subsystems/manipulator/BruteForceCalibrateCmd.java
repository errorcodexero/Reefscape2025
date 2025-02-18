package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;

public class BruteForceCalibrateCmd extends Command {
    private ManipulatorSubsystem m_ ;
    private boolean done_ ;

    public BruteForceCalibrateCmd(ManipulatorSubsystem m) {
        m_ = m ;
    }

    @Override
    public void initialize() {
        m_.setElevatorVoltage(Volts.of(-1.0)) ;
        done_ = false;
    }

    @Override
    public void execute() {
        if (m_.getElevatorVelocity().lte(MetersPerSecond.of(0.01))) {
            m_.resetElevator() ;
            m_.setElevatorVoltage(Volts.of(0.0)) ;
        }
    }

    @Override
    public boolean isFinished() {
        return done_ ;
    }
}
