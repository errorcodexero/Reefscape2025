package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ReefLevel;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.placecoral.PlaceCoralCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.WaitForCoralCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.oi.CoralSide;

public class AutoCommands {
  private AutoCommands() {
  }

  public static Command sideCoralAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub,
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
            new SetHoldingCmd(brainSub, GamePiece.CORAL)),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, false, ReefLevel.L4, CoralSide.Left),
        DriveCommands.followPathCommand("Side Coral 2", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Side Coral 3", mirroredX),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, false, ReefLevel.L4, CoralSide.Left),
        DriveCommands.followPathCommand("Side Coral 4", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Side Coral 5", mirroredX),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, false, ReefLevel.L4, CoralSide.Left),
        DriveCommands.followPathCommand("Side Coral 6", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Side Coral 7", mirroredX),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, false, ReefLevel.L4, CoralSide.Left));
  }

  public static Command algaeAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub) {
    return Commands.sequence(
        // Commands.parallel(
        //     DriveCommands.initialFollowPathCommand(driveSub, "Algae 1"),
        //     new SetHoldingCmd(brainSub, GamePiece.CORAL)),
        // new PlaceCoralCmd(driveSub, manipSub, grabberSub, brainSub, false, ReefLevel.L4, CoralSide.Left),
        // DriveCommands.followPathCommand("Algae 1.5"),
        // new CollectAlgaeReefCmd(brainSub, manipSub, grabberSub, ReefLevel.L2),
        // DriveCommands.followPathCommand("Algae 2"),
        // new DepositAlgaeCmd(grabberSub),
        // DriveCommands.followPathCommand("Algae 3"),
        // new CollectAlgaeReefCmd(brainSub, manipSub, grabberSub, ReefLevel.L3),
        // DriveCommands.followPathCommand("Algae 4"),
        // new DepositAlgaeCmd(grabberSub),
        // DriveCommands.followPathCommand("Algae 5"),
        // new CollectAlgaeReefCmd(brainSub, manipSub, grabberSub, ReefLevel.L3),
        // DriveCommands.followPathCommand("Algae 6"),
        // new DepositAlgaeCmd(grabberSub)
      );
  }

  public static Command centerCoralAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub,
      boolean mirroredX) {
    boolean mirroredY = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    if (!mirroredY) {
      mirroredX = !mirroredX;
    }
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.initialFollowPathCommand(driveSub, "Center Coral 1", mirroredX),
            new SetHoldingCmd(brainSub, GamePiece.CORAL)),
            new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, false, ReefLevel.L4, CoralSide.Left),
        DriveCommands.followPathCommand("Center Coral 2", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Center Coral 3", mirroredX),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, false, ReefLevel.L4, CoralSide.Left),
        DriveCommands.followPathCommand("Center Coral 4", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Center Coral 5", mirroredX),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, false, ReefLevel.L4, CoralSide.Left),
        DriveCommands.followPathCommand("Center Coral 6", mirroredX),
        new WaitForCoralCmd(grabberSub),
        DriveCommands.followPathCommand("Center Coral 7", mirroredX),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, false, ReefLevel.L4, CoralSide.Left)) ;
  }

  public static Command justCoralAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub) {
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.initialFollowPathCommand(driveSub, "Just Coral 1"),
            new SetHoldingCmd(brainSub, GamePiece.CORAL)),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, false, ReefLevel.L4, CoralSide.Left),
        DriveCommands.followPathCommand("Just Coral 2"));
  }
}