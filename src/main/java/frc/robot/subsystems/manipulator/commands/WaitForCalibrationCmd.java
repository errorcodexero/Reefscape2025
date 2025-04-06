package frc.robot.subsystems.manipulator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class WaitForCalibrationCmd extends Command {
    private ManipulatorSubsystem m_ ;

    public WaitForCalibrationCmd(ManipulatorSubsystem m) {
        m_ = m ;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return m_.isElevatorCalibrated() ;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
