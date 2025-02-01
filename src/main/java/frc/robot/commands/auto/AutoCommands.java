package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class AutoCommands {
    private AutoCommands() {}

    /**
     * TODO: put in numbers and other commands when they are available.
     */

    public static Command sideCoralAuto(ManipulatorSubsystem manipSub, boolean mirroredX, boolean mirroredY){
        if(mirroredY) mirroredX = !mirroredX;
        return Commands.sequence(
            Commands.parallel(
                new PathPlannerAuto("Side Coral 1", mirroredX),
                new GoToCmd(manipSub)// add positions later, L4 place
                // add has coral later
            ),
            // Place Coral command here
            Commands.parallel(
                new PathPlannerAuto("Side Coral 2", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4),
                    new GoToCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                new PathPlannerAuto("Side Coral 3", mirroredX),
                new GoToCmd(manipSub)// add positions later, L4 place
            ), 
            // Place Coral command here
            Commands.parallel(
                new PathPlannerAuto("Side Coral 4", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4),
                    new GoToCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                new PathPlannerAuto("Side Coral 3", mirroredX),
                new GoToCmd(manipSub)// add positions later, L4 place
            ), 
            // Place Coral command here
            Commands.parallel(
                new PathPlannerAuto("Side Coral 4", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4),
                    new GoToCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                new PathPlannerAuto("Side Coral 5", mirroredX),
                new GoToCmd(manipSub)// add positions later, L4 place
            )
            // Place Coral command here

        );
    }

}