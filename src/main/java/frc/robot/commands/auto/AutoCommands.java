package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ReefLevel;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.WaitForCoralInRobot;
import frc.robot.commands.robot.collectalgaereef.CollectAlgaeReefCmd;
import frc.robot.commands.robot.collectcoral.CollectCoralCmd;
import frc.robot.commands.robot.placecoral.PlaceCoralCmd;
import frc.robot.commands.robot.scorealgae.ScoreAlgaeAfter;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.oi.CoralSide;

public class AutoCommands {
  private AutoCommands() {
  }

  //
  // - Start from directly behind the reef
  // - Drive forward to the reef
  // - Place the coral on the reef
  // - Drive backwards to the starting position
  //
  public static Command oneCoralBackAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub) {
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.initialFollowPathCommand(driveSub, "Just Coral 1"),
            new SetHoldingCmd(brainSub, GamePiece.CORAL)),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, true, ReefLevel.L4, CoralSide.Left),
        DriveCommands.followPathCommand("Just Coral 2"));
  }

  //
  // - Start offset from the center position to the side where the robot will place
  // - Drive to the side of the reef and place first coral
  // - Drive to coral collect station on the side nearest the endline and collect
  // - Place a total of three corals on the reef
  //
  public static Command threeCoralSideAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub, FunnelSubsystem funnel, boolean mirroredX) {

    return Commands.sequence(
        // logState("threeCoralAuto", "Path 1"),
        Commands.parallel(
            DriveCommands.initialFollowPathCommand(driveSub, "Side Coral 1", mirroredX),
            new SetHoldingCmd(brainSub, GamePiece.CORAL)),
        // logState("threeCoralAuto", "Placing"),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, true, ReefLevel.L4, mirroredX ? CoralSide.Right : CoralSide.Left),
        // logState("threeCoralAuto", "Path 2"),
        DriveCommands.followPathCommand("Side Coral 2", mirroredX),
        // logState("threeCoralAuto", "WaitForFunnelSensor"),
        new WaitForCoralInRobot(grabberSub, funnel),
        // logState("threeCoralAuto", "Path 3/Collect"),
        Commands.parallel(
          new CollectCoralCmd(brainSub, manipSub, grabberSub),
          DriveCommands.followPathCommand("Side Coral 3", mirroredX)),
        // logState("threeCoralAuto", "Placing"),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, true, ReefLevel.L4, mirroredX ? CoralSide.Right : CoralSide.Left),
        // logState("threeCoralAuto", "Path 4"),
        DriveCommands.followPathCommand("Side Coral 4", mirroredX),
        // logState("threeCoralAuto", "Wait For Coral"),
        new WaitForCoralInRobot(grabberSub, funnel),
        // logState("threeCoralAuto", "Path 5/Collect"),
        Commands.parallel(
          new CollectCoralCmd(brainSub, manipSub, grabberSub),
          DriveCommands.followPathCommand("Side Coral 5", mirroredX)),
        // logState("threeCoralAuto", "Placing"),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, true, ReefLevel.L4, mirroredX ? CoralSide.Left : CoralSide.Right)
      );
  }

  private static Command logState(String mode, String state) {
    return Commands.runOnce(() -> Logger.recordOutput("Autos/" + mode, state));
  }

  public static Command oneCoralOneAlgaeAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub) {
    return Commands.sequence(
      
        logState("oneCoralOneAlgaeAuto", "Place"),
        DriveCommands.setPoseCommand(driveSub, new Pose2d(7.22, 3.85, Rotation2d.fromDegrees(180.0)), true),
        new SetHoldingCmd(brainSub, GamePiece.CORAL),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, true, ReefLevel.L4, CoralSide.Left),

        logState("oneCoralOneAlgaeAuto", "Backup from reef"),
        Commands.deadline(
          new WaitCommand(1.0),
          driveSub.runVelocityCmd(MetersPerSecond.of(-1.0), MetersPerSecond.of(0), RadiansPerSecond.zero())),  

        logState("oneCoralOneAlgaeAuto", "Collect Algae from reef"),
        new CollectAlgaeReefCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L2),

        logState("oneCoralOneAlgaeAuto", "driveProcessor"),
        DriveCommands.followPathCommand("Algae 2"),       
        new ScoreAlgaeAfter(driveSub, brainSub, manipSub, grabberSub)
      );
  }

  public static Command twoCoralCenterAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, 
                                        GrabberSubsystem grabberSub, FunnelSubsystem funnel, boolean mirroredX) {
    return Commands.sequence(
        DriveCommands.setPoseCommand(driveSub, new Pose2d(7.22, 3.85, Rotation2d.fromDegrees(180.0)), true),
        new SetHoldingCmd(brainSub, GamePiece.CORAL),
        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, true, ReefLevel.L4, CoralSide.Left),

        DriveCommands.followPathCommand("Center Coral 2", mirroredX),
        new WaitForCoralInRobot(grabberSub, funnel),

        Commands.parallel(
          new CollectCoralCmd(brainSub, manipSub, grabberSub),
          DriveCommands.followPathCommand("Center Coral 3", mirroredX)),

        new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, true, ReefLevel.L4, CoralSide.Right)) ;
  }
}