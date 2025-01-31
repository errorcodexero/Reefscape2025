package frc.robot.commands.gps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class EjectCmd extends Command {
    private ManipulatorSubsystem manipulator_ ;
    private GrabberSubsystem grabber_ ;

    public EjectCmd(ManipulatorSubsystem m, GrabberSubsystem g) {
        addRequirements(m, g);
        manipulator_ = m;
        grabber_ = g;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }    
}
