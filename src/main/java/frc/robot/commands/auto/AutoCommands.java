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
    private AutoCommands() {}

    /**
     * TODO: put in numbers and other commands when they are available.
     */

    public static Command sideCoralAuto(Drive driveSub, ManipulatorSubsystem manipSub, boolean mirroredX){
        boolean mirroredY = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().equals(Alliance.Red) : false;
        if(!mirroredY) {
            mirroredX = !mirroredX; 
        }

        return Commands.sequence(
            Commands.parallel(
                DriveCommands.followPathCommand("Side Coral 1", mirroredX, driveSub)//,
                //new GotoCmd(manipSub)// add positions later, L4 place
                // add has coral later
                
            ),
            // Place Coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Side Coral 2", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Side Coral 3", mirroredX)//,
                //new GotoCmd(manipSub)// add positions later, L4 place
            ), 
            // Place Coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Side Coral 4", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Side Coral 5", mirroredX)//,
                //new GotoCmd(manipSub)// add positions later, L4 place
            ), 
            // Place Coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Side Coral 6", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Side Coral 7", mirroredX)//,
                //new GotoCmd(manipSub)// add positions later, L4 place
            )
            // Place Coral command here

        );
    }

    public static Command algaeAuto(Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub){
        return Commands.sequence(
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 1", true, driveSub)//,
                ////new GotoCmd(manipSub)// add positions later, L4 place
                // add has coral later
            ),
            // Place Coral command here,
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 1.5", true)//,
                //new GotoCmd(manipSub)// add positions later, Algae L2 collect
                // Might need to do the sequence thing, but probably not
            ),
            //Collect Algae L2 here,
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 2", true),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, Scoring algae
            ),
            // Score algae here
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 3", true)//,
                //new GotoCmd(manipSub)// add positions later, L3 algae collect
            ), 
            // Collect algae L3 command here
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 4", true),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, Score algae here
            ),
            // Score algae
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 5", true)//,
                //new GotoCmd(manipSub)// add positions later, L3 collect algae
            ),
            // Algae L3 collect here
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 6", true),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, Score algae here
            )
            // Score algae here

        );
    }


    public static Command centerCoralAuto(Drive driveSub, ManipulatorSubsystem manipSub, boolean mirroredX){
        boolean mirroredY = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().equals(Alliance.Red) : false;
        if(!mirroredY) {
            mirroredX = !mirroredX; // Might not be needed
        }
        return Commands.sequence(
            Commands.parallel(
                DriveCommands.followPathCommand("Center Coral 1", mirroredX, driveSub)//,
                //new GotoCmd(manipSub)// add positions later, L4 place
                // add has coral later
            ),
            // Place Coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Center Coral 2", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Center Coral 3", mirroredX)//,
                //new GotoCmd(manipSub)// add positions later, L4 place
            ),
            // Place Coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Center Coral 4", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Center Coral 5", mirroredX)//,
                //new GotoCmd(manipSub)// add positions later, L4 place
            ),
            // Place Coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Center Coral 6", mirroredX),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, station collect
            ),
            // Wait for coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Center Coral 7", mirroredX)//,
                //new GotoCmd(manipSub)// add positions later, L4 place
            )
            // Place Coral command here
        );
    }

    public static Command justCoralAuto(Drive driveSub, ManipulatorSubsystem manipSub){

        return Commands.sequence(
            Commands.parallel(
                DriveCommands.followPathCommand("Just Coral 1", true, driveSub)//,
                //new GotoCmd(manipSub)// add positions later, L4 place
                // add has coral later
            ),
            // Place Coral command here
            Commands.parallel(
                DriveCommands.followPathCommand("Just Coral 2", true),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, station collect
            )
        );
    }

}