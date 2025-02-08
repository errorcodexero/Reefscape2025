package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class EjectCoralCmd extends SequentialCommandGroup {
    private ManipulatorSubsystem manipulator_ ;
    private GrabberSubsystem grabber_ ;

    public EjectCoralCmd(ManipulatorSubsystem m, GrabberSubsystem g) {
        manipulator_ = m;
        grabber_ = g;

        addCommands(
            new GoToCmd(manipulator_, ManipulatorConstants.Positions.kEjectAlgaeHeight, ManipulatorConstants.Positions.kEjectAlgaeAngle),
            new DepositAlgaeCmd(grabber_)) ;
    }
}
