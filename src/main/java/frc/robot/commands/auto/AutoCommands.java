package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class AutoCommands {
    private AutoCommands() {
    }

    /**
     * TODO: put in numbers and other commands when they are available.
     * NOTE: All comments are going to be replaced with their respective commands.
     */

    public static Command sideCoralAuto(Drive driveSub, ManipulatorSubsystem manipSub, boolean mirroredX) {
        boolean mirroredY = DriverStation.getAlliance().isPresent()
                ? DriverStation.getAlliance().get().equals(Alliance.Red)
                : false;
        if (!mirroredY) {
            mirroredX = !mirroredX;
        }

        return Commands.sequence(
                Commands.parallel(
                        DriveCommands.initialFollowPathCommand(driveSub, "Side Coral 1", mirroredX)// ,
                // add has coral later
                ),
                // new GotoCmd(manipSub)// add positions later, L4 place
                // Place Coral command here
                // new GotoCmd(manipSub) // add positions later, station collect
                DriveCommands.followPathCommand("Side Coral 2", mirroredX),
                // Wait for coral command here
                DriveCommands.followPathCommand("Side Coral 3", mirroredX),
                // new GotoCmd(manipSub)// add positions later, L4 place
                // Place Coral command here
                // new GotoCmd(manipSub)// add positions later, station collect
                DriveCommands.followPathCommand("Side Coral 4", mirroredX),
                // Wait for coral command here
                DriveCommands.followPathCommand("Side Coral 5", mirroredX),
                // new GotoCmd(manipSub)// add positions later, L4 place
                // Place Coral command here
                // new GotoCmd(manipSub)// add positions later, station collect
                DriveCommands.followPathCommand("Side Coral 6", mirroredX),
                // Wait for coral command here
                DriveCommands.followPathCommand("Side Coral 7", mirroredX)// ,
        // new GotoCmd(manipSub)// add positions later, L4 place
        // Place Coral command here
        // new GotoCmd(manipSub)// add positions later, station collect

        );
    }

    public static Command algaeAuto(Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub) {
        return Commands.sequence(
                Commands.parallel(
                        DriveCommands.initialFollowPathCommand(driveSub, "Algae 1")// ,
                // add has coral later
                ),
                // new GotoCmd(manipSub)// add positions later, L4 place
                // Place Coral command here,
                Commands.parallel(
                        DriveCommands.followPathCommand("Algae 1.5")// ,
                // new GotoCmd(manipSub)// add positions later, Algae L2 collect from L4
                ),
                // Collect Algae L2 here,
                // new GotoCmd(manipSub)// add positions later, Scoring algae
                DriveCommands.followPathCommand("Algae 2"),
                // Score algae here
                DriveCommands.followPathCommand("Algae 3"),
                // new GotoCmd(manipSub)// add positions later, L3 algae collect
                // Collect algae L3 command here
                // new GotoCmd(manipSub)// add positions later, Score algae here
                DriveCommands.followPathCommand("Algae 4"),
                // Score algae
                DriveCommands.followPathCommand("Algae 5"),
                // new GotoCmd(manipSub)// add positions later, L3 collect algae
                // Algae L3 collect here
                // new GotoCmd(manipSub)// add positions later, Score algae here
                DriveCommands.followPathCommand("Algae 6")// ,
        // Score algae here
        );
    }

    public static Command centerCoralAuto(Drive driveSub, ManipulatorSubsystem manipSub, boolean mirroredX) {
        boolean mirroredY = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        if (!mirroredY) {
            mirroredX = !mirroredX;
        }
        return Commands.sequence(
                Commands.parallel(
                        DriveCommands.initialFollowPathCommand(driveSub, "Center Coral 1", mirroredX)// ,
                // add has coral later
                ),
                // new GotoCmd(manipSub)// add positions later, L4 place
                // Place Coral command here
                // new GotoCmd(manipSub)// add positions later, station collect
                DriveCommands.followPathCommand("Center Coral 2", mirroredX),
                // Wait for coral command here
                DriveCommands.followPathCommand("Center Coral 3", mirroredX),
                // new GotoCmd(manipSub)// add positions later, L4 place
                // Place Coral command here
                // new GotoCmd(manipSub)// add positions later, station collect
                DriveCommands.followPathCommand("Center Coral 4", mirroredX),
                // Wait for coral command here
                DriveCommands.followPathCommand("Center Coral 5", mirroredX),
                // new GotoCmd(manipSub)// add positions later, L4 place
                // Place Coral command here
                // new GotoCmd(manipSub)// add positions later, station collect
                DriveCommands.followPathCommand("Center Coral 6", mirroredX),
                // Wait for coral command here
                DriveCommands.followPathCommand("Center Coral 7", mirroredX)// ,
        // new GotoCmd(manipSub)// add positions later, L4 place
        // Place Coral command here
        // new GotoCmd(manipSub)// add positions later, station collect
        );
    }

    public static Command justCoralAuto(Drive driveSub, ManipulatorSubsystem manipSub) {
        return Commands.sequence(
                Commands.parallel(
                        DriveCommands.initialFollowPathCommand(driveSub, "Just Coral 1")// ,
                // add has coral later
                ),
                // new GotoCmd(manipSub)// add positions later, L4 place
                // Place Coral command here
                // new GotoCmd(manipSub)// add positions later, station collect
                DriveCommands.followPathCommand("Just Coral 2"));
    }

}