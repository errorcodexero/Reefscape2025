package frc.robot.commands.gps;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class EjectAlgaeCmd extends SequentialCommandGroup {
    private ManipulatorSubsystem manipulator_ ;
    private GrabberSubsystem grabber_ ;

    public EjectAlgaeCmd(ManipulatorSubsystem m, GrabberSubsystem g) {
        manipulator_ = m;
        grabber_ = g;

        addCommands(
            new GoToCmd(m, null, null)
        );
    }
}
