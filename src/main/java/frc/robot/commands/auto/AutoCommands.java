package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Height;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.placecoral.PlaceCoralAutoCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.CollectAlgaeAutoCmd;
import frc.robot.subsystems.grabber.commands.DepositAlgaeCmd;
import frc.robot.subsystems.grabber.commands.WaitForCoralCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class AutoCommands {
  private AutoCommands() {
  }

  /**
   * TODO: put in numbers and other commands when they are available.
   * NOTE: All comments are going to be replaced with their respective commands.
   */

  public static Command sideCoralAuto(Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub,
      FunnelSubsystem funnelSub, boolean mirroredX) {
    boolean mirroredY = DriverStation.getAlliance().isPresent()
        ? DriverStation.getAlliance().get().equals(Alliance.Red)
        : false;
    if (!mirroredY) {
      mirroredX = !mirroredX;
    }

    return Commands.sequence(
        Commands.parallel(
            DriveCommands.initialFollowPathCommand(driveSub, "Side Coral 1", mirroredX),
            grabberSub.setHasCoralCmd(true)),
        new PlaceCoralAutoCmd(manipSub, grabberSub, Height.L4),
        DriveCommands.followPathCommand("Side Coral 2", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Side Coral 3", mirroredX),
        new PlaceCoralAutoCmd(manipSub, grabberSub, Height.L4),
        DriveCommands.followPathCommand("Side Coral 4", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Side Coral 5", mirroredX),
        new PlaceCoralAutoCmd(manipSub, grabberSub, Height.L4),
        DriveCommands.followPathCommand("Side Coral 6", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Side Coral 7", mirroredX),
        new PlaceCoralAutoCmd(manipSub, grabberSub, Height.L4)

    );
  }

  public static Command algaeAuto(Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub) {
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.initialFollowPathCommand(driveSub, "Algae 1"),
            grabberSub.setHasCoralCmd(true)),
        new PlaceCoralAutoCmd(manipSub, grabberSub, Height.L4),
        DriveCommands.followPathCommand("Algae 1.5"),
        new CollectAlgaeAutoCmd(grabberSub, manipSub, Height.L2),
        DriveCommands.followPathCommand("Algae 2"),
        new DepositAlgaeCmd(grabberSub),
        DriveCommands.followPathCommand("Algae 3"),
        new CollectAlgaeAutoCmd(grabberSub, manipSub, Height.L3),
        DriveCommands.followPathCommand("Algae 4"),
        new DepositAlgaeCmd(grabberSub),
        DriveCommands.followPathCommand("Algae 5"),
        new CollectAlgaeAutoCmd(grabberSub, manipSub, Height.L3),
        DriveCommands.followPathCommand("Algae 6"),
        new DepositAlgaeCmd(grabberSub));
  }

  public static Command centerCoralAuto(Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub,
      boolean mirroredX) {
    boolean mirroredY = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    if (!mirroredY) {
      mirroredX = !mirroredX;
    }
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.initialFollowPathCommand(driveSub, "Center Coral 1", mirroredX),
            grabberSub.setHasCoralCmd(true)),
        new PlaceCoralAutoCmd(manipSub, grabberSub, Height.L4),
        DriveCommands.followPathCommand("Center Coral 2", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Center Coral 3", mirroredX),
        new PlaceCoralAutoCmd(manipSub, grabberSub, Height.L4),
        DriveCommands.followPathCommand("Center Coral 4", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Center Coral 5", mirroredX),
        new PlaceCoralAutoCmd(manipSub, grabberSub, Height.L4),
        DriveCommands.followPathCommand("Center Coral 6", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Center Coral 7", mirroredX),
        new PlaceCoralAutoCmd(manipSub, grabberSub, Height.L4));
  }

  public static Command justCoralAuto(Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub) {
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.initialFollowPathCommand(driveSub, "Just Coral 1"),
            grabberSub.setHasCoralCmd(true)),
        new PlaceCoralAutoCmd(manipSub, grabberSub, Height.L4),
        DriveCommands.followPathCommand("Just Coral 2"));
  }

}