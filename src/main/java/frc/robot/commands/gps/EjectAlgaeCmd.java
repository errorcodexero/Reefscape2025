package frc.robot.commands.gps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class EjectAlgaeCmd extends Command {
    private ManipulatorSubsystem manipulator_ ;
    private GrabberSubsystem grabber_ ;

    public EjectAlgaeCmd(ManipulatorSubsystem m, GrabberSubsystem g) {
        manipulator_ = m;
        grabber_ = g;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
