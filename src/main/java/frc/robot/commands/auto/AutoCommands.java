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
                new PathPlannerAuto("Side Coral 5", mirroredX),
                new GoToCmd(manipSub)// add positions later, L4 place
            ), 
            // Place Coral command here
            Commands.parallel(
                new PathPlannerAuto("Side Coral 6", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4),
                    new GoToCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                new PathPlannerAuto("Side Coral 7", mirroredX),
                new GoToCmd(manipSub)// add positions later, L4 place
            )
            // Place Coral command here

        );
    }

    public static Command algaeAuto(ManipulatorSubsystem manipSub, boolean mirroredY){
        return Commands.sequence(
            Commands.parallel(
                new PathPlannerAuto("Algae 1", mirroredY),
                new GoToCmd(manipSub)// add positions later, L4 place
                // add has coral later
            ),
            // Place Coral command here,
            Commands.parallel(
                new PathPlannerAuto("Algae 1.5", mirroredY),
                new GoToCmd(manipSub)// add positions later, Algae L2 collect
                // Might need to do the sequence thing, but probably not
            ),
            //Collect Algae L2 here,
            Commands.parallel(
                new PathPlannerAuto("Algae 2", mirroredY),
                Commands.sequence(
                    Commands.waitSeconds(0.4),
                    new GoToCmd(manipSub)
                )
                // add positions later, Scoring algae
            ),
            // Score algae here
            Commands.parallel(
                new PathPlannerAuto("Algae 3", mirroredY),
                new GoToCmd(manipSub)// add positions later, L3 algae collect
            ), 
            // Collect algae L3 command here
            Commands.parallel(
                new PathPlannerAuto("Algae 4", mirroredY),
                Commands.sequence(
                    Commands.waitSeconds(0.4),
                    new GoToCmd(manipSub)
                )
                // add positions later, Score algae here
            ),
            // Score algae
            Commands.parallel(
                new PathPlannerAuto("Algae 5", mirroredY),
                new GoToCmd(manipSub)// add positions later, L3 collect algae
            ),
            // Algae L3 collect here
            Commands.parallel(
                new PathPlannerAuto("Algae 6", mirroredY),
                Commands.sequence(
                    Commands.waitSeconds(0.4),
                    new GoToCmd(manipSub)
                )
                // add positions later, Score algae here
            )
            // Score algae here

        );
    }


    public static Command centerCoralAuto(ManipulatorSubsystem manipSub, boolean mirroredX, boolean mirroredY){
        if(mirroredY) mirroredX =!mirroredX;
        return Commands.sequence(
            Commands.parallel(
                new PathPlannerAuto("Center Coral 1", mirroredX),
                new GoToCmd(manipSub)// add positions later, L4 place
                // add has coral later
            ),
            // Place Coral command here
            Commands.parallel(
                new PathPlannerAuto("Center Coral 2", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4),
                    new GoToCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                new PathPlannerAuto("Center Coral 3", mirroredX),
                new GoToCmd(manipSub)// add positions later, L4 place
            ),
            // Place Coral command here
            Commands.parallel(
                new PathPlannerAuto("Center Coral 4", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4),
                    new GoToCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                new PathPlannerAuto("Center Coral 5", mirroredX),
                new GoToCmd(manipSub)// add positions later, L4 place
            )
            // Place Coral command here
        );
    }

}