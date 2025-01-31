package frc.robot.commands.gps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class AbortCmd extends Command {
    public AbortCmd(Drive db, ManipulatorSubsystem m, GrabberSubsystem g, ClimberSubsystem c) {
        addRequirements(db, m, g, c);
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
