package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ReefLevel;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.NullCmd;
import frc.robot.commands.robot.WaitForCoralInRobot;
import frc.robot.commands.robot.collectalgaereef.CollectAlgaeReefCmd;
import frc.robot.commands.robot.collectcoral.CollectCoralCmd;
import frc.robot.commands.robot.placecoral.PlaceCoralCmd;
import frc.robot.commands.robot.scorealgae.ScoreAlgaeAfter;
import frc.robot.commands.misc.StateCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.oi.CoralSide;

public class AutoCommands {

    private final static boolean kDebug = true ;
    private final static boolean kDriveWhileRaising = true ;
    private final static Time DelayBeforeDriving = Milliseconds.of(20) ;

    private AutoCommands() {
    }

    private static void addToSequence(SequentialCommandGroup seq, Command cmd) {
        if (cmd != null) {
            seq.addCommands(cmd);
        }
    }

    //
    // - Start from directly behind the reef
    // - Drive forward to the reef
    // - Place the coral on the reef
    // - Drive backwards to the starting position
    //
    public static Command oneCoralAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub) {
        final String modename = "oneCoralBackAuto" ;

        SequentialCommandGroup seq = new SequentialCommandGroup();

        addToSequence(seq, logState(modename, "Start"));
        addToSequence(seq, new CollectCoralCmd(brainSub, manipSub, grabberSub, false)) ;

        addToSequence(seq, logState(modename, "Place-1"));
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4, CoralSide.Left)) ;
        
        addToSequence(seq, DriveCommands.followPathCommand("Just Coral 2")) ;
        addToSequence(seq, logState(modename, "done")) ;

        return seq ;
    }

    private static boolean hasCoral(BrainSubsystem brain) {
        return brain.gp() == GamePiece.CORAL;
    }

    //
    // - Start offset from the center position to the side where the robot will place
    // - Drive to the side of the reef and place first coral
    // - Drive to coral collect station on the side nearest the endline and collect
    // - Place a total of three corals on the reef
    //
    public static Command threeCoralSideAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub, FunnelSubsystem funnel, boolean mirroredX) {
        final String modename = "threeCoralSideAuto" ;

        SequentialCommandGroup seq = new SequentialCommandGroup();

        addToSequence(seq, logState(modename, "Start"));

        //
        // Drive first path and be sure coral is ready to place
        //
        addToSequence(seq,
            Commands.parallel(
                DriveCommands.initialFollowPathCommand(driveSub, "Side Coral 1", mirroredX),
                new CollectCoralCmd(brainSub, manipSub, grabberSub, false))) ;

        // 
        // Place first coral
        //
        addToSequence(seq, logState(modename, "Place 1st"));
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4, mirroredX ? CoralSide.Right : CoralSide.Left, false, AutoCommands.kDriveWhileRaising)) ;

        //
        // Start driving to collect and lowering the elevator in parallel
        //
        addToSequence(seq, logState(modename, "Drive to Collect 2nd"));
        addToSequence(seq,
                Commands.parallel(
                    Commands.sequence(
                        new WaitCommand(AutoCommands.DelayBeforeDriving),
                        DriveCommands.followPathCommand("Side Coral 2", mirroredX)),
                    new GoToCmd(manipSub, ManipulatorConstants.Elevator.Positions.kCollect, ManipulatorConstants.Arm.Positions.kCollect))) ;

        //
        // Wait for coral to pass through the funnel
        //          
        addToSequence(seq, logState(modename, "Wait For 2nd"));
        addToSequence(seq, new WaitForCoralInRobot(grabberSub, funnel)) ;

        //
        // Drive to place position while collecting coral.  The path ends a few feet away with a forware
        // velocity.
        //
        addToSequence(seq, logState(modename, "Drive to Place 2nd")) ;
        addToSequence(seq,
                Commands.parallel(
                    new CollectCoralCmd(brainSub, manipSub, grabberSub, false),
                    Commands.sequence(
                        DriveCommands.followPathCommand("Side Coral 3", mirroredX),
                        new ConditionalCommand(
                            new NullCmd(), 
                            driveSub.stopCmd(),
                            () -> AutoCommands.hasCoral(brainSub))))) ;

        //
        // Place the second coral
        //
        addToSequence(seq, logState(modename, "Place 2nd")) ;
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4, mirroredX ? CoralSide.Right : CoralSide.Left, false, AutoCommands.kDriveWhileRaising)) ;

        //
        // Start driving to collect and lowering the elevator in parallel
        //
        addToSequence(seq, logState(modename, "Drive to Collect 3rd")) ;
        addToSequence(seq, 
                Commands.parallel(
                    Commands.sequence(
                        new WaitCommand(DelayBeforeDriving),
                        DriveCommands.followPathCommand("Side Coral 4", mirroredX)),
                    new GoToCmd(manipSub, ManipulatorConstants.Elevator.Positions.kCollect, ManipulatorConstants.Arm.Positions.kCollect))) ;

        //
        // Wait for coral to pass through the funnel
        //          
        addToSequence(seq, logState(modename, "Wait For 3rd")) ;
        addToSequence(seq, new WaitForCoralInRobot(grabberSub, funnel)) ;

        //
        // Drive to place position while collecting coral.  The path ends a few feet away with a velocity
        // of 1.5 m/s and the place coral below takes over
        //        
        addToSequence(seq, logState(modename, "Drive to Place 3rd")) ;
        addToSequence(seq,
                Commands.parallel(
                    new CollectCoralCmd(brainSub, manipSub, grabberSub, false),
                    Commands.sequence(
                        DriveCommands.followPathCommand("Side Coral 5", mirroredX),
                        new ConditionalCommand(
                            new NullCmd(), 
                            driveSub.stopCmd(),
                            () -> AutoCommands.hasCoral(brainSub))))) ;

        //
        // Place the third coral
        //
        addToSequence(seq, logState(modename, "Place 3rd")) ;
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4, mirroredX ? CoralSide.Left : CoralSide.Right, true, AutoCommands.kDriveWhileRaising)) ;

        addToSequence(seq, logState(modename, "done")) ;

        return seq ;
    }

    private static Command logState(String mode, String state) {
        return kDebug ? new StateCmd("Autos/" + mode, state) : null ;
    }

    public static Command oneCoralOneAlgaeAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub) {
        final String modename = "oneCoralOneAlgaeAuto" ;
        SequentialCommandGroup seq = new SequentialCommandGroup();


        addToSequence(seq, logState(modename, "Start"));
        addToSequence(seq, DriveCommands.setPoseCommand(driveSub, new Pose2d(7.22, 3.85, Rotation2d.fromDegrees(180.0)), true)) ;
        addToSequence(seq, new SetHoldingCmd(brainSub, GamePiece.CORAL)) ;
        addToSequence(seq, logState(modename, "Place Coral"));
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4, CoralSide.Left, false, AutoCommands.kDriveWhileRaising)) ;

        addToSequence(seq, logState(modename, "Backup From Reef"));
        addToSequence(seq,
            Commands.parallel(
                Commands.deadline(
                    new WaitCommand(0.10),
                    driveSub.runVelocityCmd(MetersPerSecond.of(-1.0), MetersPerSecond.of(0), RadiansPerSecond.zero())),
                new GoToCmd(manipSub, ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL2, ManipulatorConstants.Arm.Positions.kAlgaeReefCollectL2))) ;
        addToSequence(seq, logState(modename, "Stop"));
        addToSequence(seq, driveSub.stopCmd()) ;

        addToSequence(seq, logState(modename, "Collect Algae"));
        addToSequence(seq, new CollectAlgaeReefCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L2, true, true)) ;

        addToSequence(seq, logState(modename, "Drive Processor")) ;
        addToSequence(seq, DriveCommands.followPathCommand("Algae 2")) ;
        addToSequence(seq, new ScoreAlgaeAfter(driveSub, brainSub, manipSub, grabberSub)) ;

        addToSequence(seq, logState(modename, "done")) ;    

        return seq ;
    }

    public static Command twoCoralCenterAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, 
                                                                                GrabberSubsystem grabberSub, FunnelSubsystem funnel, boolean mirroredX) {

        final String modename = "twoCoralCenterAuto" ;

        SequentialCommandGroup seq = new SequentialCommandGroup();

        addToSequence(seq, logState(modename, "Place 1st"));
        addToSequence(seq, DriveCommands.setPoseCommand(driveSub, new Pose2d(7.22, 3.85, Rotation2d.fromDegrees(180.0)), true)) ;
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4, CoralSide.Left, false, AutoCommands.kDriveWhileRaising));

        addToSequence(seq, logState(modename, "Drive to Collect 2nd"));
        addToSequence(seq, DriveCommands.followPathCommand("Center Coral 2", mirroredX)) ;

        addToSequence(seq, logState(modename, "Wait For 2nd"));
        addToSequence(seq, new WaitForCoralInRobot(grabberSub, funnel)) ;

        addToSequence(seq, logState(modename, "Drive to Place 2nd"));
        addToSequence(seq,
                Commands.parallel(
                    new CollectCoralCmd(brainSub, manipSub, grabberSub, false),
                    DriveCommands.followPathCommand("Center Coral 3", mirroredX))) ;

        addToSequence(seq, logState(modename, "Place 2nd"));
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4, CoralSide.Right, true, AutoCommands.kDriveWhileRaising)) ;
        addToSequence(seq, logState(modename, "done")) ;

        return seq ;
    }
}