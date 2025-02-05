package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class AutoCommands {
    private AutoCommands() {}

    /**
     * TODO: put in numbers and other commands when they are available.
     */

    public static Command sideCoralAuto(Drive driveSub, ManipulatorSubsystem manipSub, boolean mirroredX, boolean mirroredY){
        Distance startX = Meters.of(8.050);
        Distance startY = Meters.of(2.800);
        Angle startAngle = Rotations.of(0.5);
        // 4.015 (y) 8.030
        // 8.775 (x) 17.550
        if(mirroredX) {
            startY = Meters.of(8.030 - 2.800); 
        }
        if(mirroredY) {
            startX = Meters.of(17.550 - 8.050);
            startAngle = Rotations.of(0);
            //mirroredX = !mirroredX; /// Might not be needed
        }

        Pose2d startPose = new Pose2d(new Translation2d(startX, startY), new Rotation2d(startAngle));
        return Commands.sequence(
            Commands.parallel(
                DriveCommands.followPathCommand("Side Coral 1", mirroredX),
                //new GotoCmd(manipSub)// add positions later, L4 place
                // add has coral later
                DriveCommands.setPoseCommand(driveSub, startPose)
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

    public static Command algaeAuto(Drive driveSub, ManipulatorSubsystem manipSub, boolean mirroredY){
        Distance startX = Meters.of(8.050);
        Distance startY = Meters.of(3.850);
        Angle startAngle = Rotations.of(0.5);
        // 4.015 (y) 8.030
        // 8.775 (x) 17.550
        if(mirroredY) {
            startY = Meters.of(8.030 - 3.850); 
            startAngle = Rotations.of(0);
            startX = Meters.of(17.550 - 8.050);
        }
        Pose2d startPose = new Pose2d(new Translation2d(startX, startY), new Rotation2d(startAngle));
        return Commands.sequence(
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 1", mirroredY), // might not be needed, might just be false
                ////new GotoCmd(manipSub)// add positions later, L4 place
                // add has coral later\
                DriveCommands.setPoseCommand(driveSub, startPose)
            ),
            // Place Coral command here,
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 1.5", mirroredY)//,
                //new GotoCmd(manipSub)// add positions later, Algae L2 collect
                // Might need to do the sequence thing, but probably not
            ),
            //Collect Algae L2 here,
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 2", mirroredY),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, Scoring algae
            ),
            // Score algae here
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 3", mirroredY)//,
                //new GotoCmd(manipSub)// add positions later, L3 algae collect
            ), 
            // Collect algae L3 command here
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 4", mirroredY),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, Score algae here
            ),
            // Score algae
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 5", mirroredY)//,
                //new GotoCmd(manipSub)// add positions later, L3 collect algae
            ),
            // Algae L3 collect here
            Commands.parallel(
                DriveCommands.followPathCommand("Algae 6", mirroredY),
                Commands.sequence(
                    Commands.waitSeconds(0.4)//,
                    //new GotoCmd(manipSub)
                )
                // add positions later, Score algae here
            )
            // Score algae here

        );
    }


    public static Command centerCoralAuto(Drive driveSub, ManipulatorSubsystem manipSub, boolean mirroredX, boolean mirroredY){
        Distance startX = Meters.of(8.050);
        Distance startY = Meters.of(3.850);
        Angle startAngle = Rotations.of(0.5);
        // 4.015 (y) 8.030
        // 8.775 (x) 17.550
        if(mirroredX) {
            startY = Meters.of(8.030 - 3.850); 
        }
        if(mirroredY) {
            startX = Meters.of(17.550 - 8.050);
            startAngle = Rotations.of(0);
            //mirroredX = !mirroredX; /// Might not be needed
        }

        Pose2d startPose = new Pose2d(new Translation2d(startX, startY), new Rotation2d(startAngle));
        return Commands.sequence(
            Commands.parallel(
                DriveCommands.followPathCommand("Center Coral 1", mirroredX),
                //new GotoCmd(manipSub)// add positions later, L4 place
                // add has coral later
                DriveCommands.setPoseCommand(driveSub, startPose)
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
            )
            // Place Coral command here
        );
    }

}