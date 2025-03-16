package frc.robot.commands.auto;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ReefLevel;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.misc.StateCmd;
import frc.robot.commands.robot.NullCmd;
import frc.robot.commands.robot.WaitForCoralInRobot;
import frc.robot.commands.robot.algaenet.AlgaeNetWhileMovingCmd;
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
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.subsystems.oi.CoralSide;

public class AutoCommands {

    private final static boolean kDebug = true;
    private final static Time DelayBeforeDriving = Milliseconds.of(0);

    private AutoCommands() {
    }

    private static void addToSequence(SequentialCommandGroup seq, Command cmd) {
        if (cmd != null) {
            seq.addCommands(cmd);
        }
    }

    private static boolean hasCoral(BrainSubsystem brain) {
        return brain.gp() == GamePiece.CORAL;
    }

    private static Command logState(String mode, String state) {
        return kDebug ? new StateCmd("Autos/" + mode, state) : null;
    }

    public static AutoModeBaseCmd twoCoralSideAuto(BrainSubsystem brainSub, Drive driveSub, ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub, FunnelSubsystem funnelSub, boolean mirroredX) {
        final String modename = "twoCoralSideAuto" ;

        Optional<PathPlannerPath> path = DriveCommands.findPath("TwoCoralSide1", mirroredX) ;
        if (!path.isPresent()) {
            return new AutoModeBaseCmd("empty") ;
        }

        AutoModeBaseCmd seq = new AutoModeBaseCmd("twoCoral", path.get()) ;

        addToSequence(seq, logState(modename, "Start"));

        //
        // Drive first path and be sure coral is ready to place
        //
        addToSequence(seq,
            Commands.parallel(
                DriveCommands.followPathCommand("TwoCoralSide1", mirroredX),
                new SetHoldingCmd(brainSub, GamePiece.CORAL)
            )
        ) ;

        // 
        // Place first coral
        //
        addToSequence(seq, logState(modename, "Place 1st"));
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4, mirroredX ? CoralSide.Right : CoralSide.Left, false)) ;

        //
        // Start driving to collect and lowering the elevator in parallel
        //
        addToSequence(seq, logState(modename, "Drive to Collect 2nd"));
        addToSequence(seq,
                Commands.parallel(
                    Commands.sequence(
                        new WaitCommand(AutoCommands.DelayBeforeDriving),
                        DriveCommands.followPathCommand("TwoCoralSide2", mirroredX)),
                    new GoToCmd(manipSub, ManipulatorConstants.Elevator.Positions.kCollect, ManipulatorConstants.Arm.Positions.kCollect))) ;

        //
        // Wait for coral to pass through the funnel
        //          
        addToSequence(seq, logState(modename, "Wait For 2nd"));
        addToSequence(seq, new WaitForCoralInRobot(grabberSub, funnelSub)) ;

        //
        // Drive to place position while collecting coral.  The path ends a few feet away with a forware
        // velocity.
        //
        addToSequence(seq, logState(modename, "Drive to Place 2nd")) ;
        addToSequence(seq,
                Commands.parallel(
                    new CollectCoralCmd(brainSub, manipSub, funnelSub, grabberSub, false),
                    Commands.sequence(
                        DriveCommands.followPathCommand("ThreeCoral3", mirroredX),
                        new ConditionalCommand(
                            new NullCmd(), 
                            driveSub.stopCmd(),
                            () -> AutoCommands.hasCoral(brainSub))))) ;

        //
        // Place the second coral
        //
        addToSequence(seq, logState(modename, "Place 2nd")) ;
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4, mirroredX ? CoralSide.Right : CoralSide.Left, true)) ;

        addToSequence(seq, logState(modename, "done")) ;

        return seq ;
    }


    //
    // - Start offset from the center position to the side where the robot will
    // place
    // - Drive to the side of the reef and place first coral
    // - Drive to coral collect station on the side nearest the endline and collect
    // - Place a total of three corals on the reef
    //
    public static AutoModeBaseCmd threeCoralSideAuto(BrainSubsystem brainSub, Drive driveSub,
            ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub, FunnelSubsystem funnelSub, boolean mirroredX) {
        final String modename = "threeCoralSideAuto";

        Optional<PathPlannerPath> path = DriveCommands.findPath("ThreeCoral1", mirroredX);
        if (!path.isPresent()) {
            return new AutoModeBaseCmd("empty");
        }

        AutoModeBaseCmd seq = new AutoModeBaseCmd("threeCoral", path.get());

        addToSequence(seq, logState(modename, "Start"));

        //
        // Drive first path and be sure coral is ready to place
        //
        addToSequence(seq,
                Commands.parallel(
                        DriveCommands.followPathCommand("ThreeCoral1", mirroredX),
                        new SetHoldingCmd(brainSub, GamePiece.CORAL)));

        //
        // Place first coral
        //
        addToSequence(seq, logState(modename, "Place 1st"));
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4,
                mirroredX ? CoralSide.Right : CoralSide.Left, false));

        //
        // Start driving to collect and lowering the elevator in parallel
        //
        addToSequence(seq, logState(modename, "Drive to Collect 2nd"));
        addToSequence(seq,
                Commands.parallel(
                        Commands.sequence(
                                new WaitCommand(AutoCommands.DelayBeforeDriving),
                                DriveCommands.followPathCommand("ThreeCoral2", mirroredX)),
                        new GoToCmd(manipSub, ManipulatorConstants.Elevator.Positions.kCollect,
                                ManipulatorConstants.Arm.Positions.kCollect)));

        //
        // Wait for coral to pass through the funnel
        //
        addToSequence(seq, logState(modename, "Wait For 2nd"));
        // Start funnel immidiately
        addToSequence(seq, Commands.parallel(
                new CollectCoralCmd(brainSub, manipSub, funnelSub, grabberSub, false),
                Commands.sequence(
                        new WaitForCoralInRobot(grabberSub, funnelSub),
                        //
                        // Drive to place position while collecting coral. The path ends a few feet away
                        // with a forward
                        // velocity.
                        //
                        logState(modename, "Drive to Place 2nd"),
                        DriveCommands.followPathCommand("ThreeCoral3", mirroredX),
                        new ConditionalCommand(
                                new NullCmd(),
                                driveSub.stopCmd(),
                                () -> AutoCommands.hasCoral(brainSub)))));

        //
        // Place the second coral
        //
        addToSequence(seq, logState(modename, "Place 2nd"));
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4,
                mirroredX ? CoralSide.Right : CoralSide.Left, false));

        //
        // Start driving to collect and lowering the elevator in parallel
        //
        addToSequence(seq, logState(modename, "Drive to Collect 3rd"));
        addToSequence(seq,
                Commands.parallel(
                        Commands.sequence(
                                new WaitCommand(DelayBeforeDriving),
                                DriveCommands.followPathCommand("ThreeCoral4", mirroredX)),
                        new GoToCmd(manipSub, ManipulatorConstants.Elevator.Positions.kCollect,
                                ManipulatorConstants.Arm.Positions.kCollect)));

        //
        // Wait for coral to pass through the funnel
        //
        addToSequence(seq, logState(modename, "Wait For 3rd"));
        // Start funnel immidiately
        addToSequence(seq, Commands.parallel(
                new CollectCoralCmd(brainSub, manipSub, funnelSub, grabberSub, false),
                Commands.sequence(
                        new WaitForCoralInRobot(grabberSub, funnelSub),
                        //
                        // Drive to place position while collecting coral. The path ends a few feet away
                        // with a forward
                        // velocity.
                        //
                        logState(modename, "Drive to Place 3rd"),
                        DriveCommands.followPathCommand("ThreeCoral5", mirroredX),
                        new ConditionalCommand(
                                new NullCmd(),
                                driveSub.stopCmd(),
                                () -> AutoCommands.hasCoral(brainSub)))));
        //
        // Place the third coral
        //
        addToSequence(seq, logState(modename, "Place 3rd"));
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4,
                mirroredX ? CoralSide.Left : CoralSide.Right, true));

        addToSequence(seq, logState(modename, "done"));

        return seq;
    }

    public static AutoModeBaseCmd oneCoralOneAlgaeProcessorAuto(BrainSubsystem brainSub, Drive driveSub,
            ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub) {
        final String modename = "oneCoralOneAlgaeProcessorAuto";

        Pose2d p = new Pose2d(7.84, 3.94, Rotation2d.fromDegrees(180.0));
        AutoModeBaseCmd seq = new AutoModeBaseCmd("oneCoralOneAlgaeProcessor", p);

        addToSequence(seq, logState(modename, "Start"));
        addToSequence(seq, new SetHoldingCmd(brainSub, GamePiece.CORAL));
        addToSequence(seq, logState(modename, "Place Coral"));
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4, CoralSide.Left,
                false));

        addToSequence(seq,
                new ConditionalCommand(new NullCmd(), new WaitCommand(Seconds.of(15.0)), () -> brainSub.placedOk()));

        addToSequence(seq, logState(modename, "Backup From Reef"));
        addToSequence(seq,
                Commands.parallel(
                        DriveCommands.followPathCommand("ProcessorAlgaeBackup"),
                        new GoToCmd(manipSub, ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL2,
                                ManipulatorConstants.Arm.Positions.kAlgaeReefCollectL2)));
        addToSequence(seq, logState(modename, "Stop"));
        addToSequence(seq, driveSub.stopCmd());

        addToSequence(seq, logState(modename, "Collect Algae"));
        addToSequence(seq, new CollectAlgaeReefCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L2, true, true));

        addToSequence(seq, logState(modename, "Drive Processor"));
        addToSequence(seq, DriveCommands.followPathCommand("ProcessorAlgaeProcessor"));
        addToSequence(seq, new ScoreAlgaeAfter(driveSub, brainSub, manipSub, grabberSub));

        addToSequence(seq, logState(modename, "done"));

        return seq;
    }

    public static AutoModeBaseCmd oneCoralOneAlgaeBargeAuto(BrainSubsystem brainSub, Drive driveSub,
            ManipulatorSubsystem manipSub, GrabberSubsystem grabberSub) {
        final String modename = "oneCoralOneAlgaeBargeAuto";

        Pose2d p = new Pose2d(7.84, 3.94, Rotation2d.fromDegrees(180.0));
        AutoModeBaseCmd seq = new AutoModeBaseCmd("oneCoralOneAlgaeBarge", p);

        addToSequence(seq, logState(modename, "Start"));
        addToSequence(seq, new SetHoldingCmd(brainSub, GamePiece.CORAL));
        addToSequence(seq, logState(modename, "Place Coral"));
        addToSequence(seq, new PlaceCoralCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L4, CoralSide.Left, false));

        addToSequence(seq,
                new ConditionalCommand(new NullCmd(), new WaitCommand(Seconds.of(15.0)), () -> brainSub.placedOk()));

        addToSequence(seq, logState(modename, "Backup From Reef"));
        addToSequence(seq,
                Commands.parallel(
                        DriveCommands.followPathCommand("BargeBackup"),
                        new GoToCmd(manipSub, ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL2,
                                ManipulatorConstants.Arm.Positions.kAlgaeReefCollectL2)));
        addToSequence(seq, logState(modename, "Stop"));
        addToSequence(seq, driveSub.stopCmd());

        addToSequence(seq, logState(modename, "Collect Algae"));
        addToSequence(seq, new CollectAlgaeReefCmd(brainSub, driveSub, manipSub, grabberSub, ReefLevel.L2, true, true));

        addToSequence(seq, logState(modename, "Drive Barge"));
        addToSequence(seq, DriveCommands.followPathCommand("BargeBarge"));
        addToSequence(seq, new AlgaeNetWhileMovingCmd(brainSub, driveSub, manipSub, grabberSub)) ;

        addToSequence(seq, logState(modename, "done"));

        return seq;
    }    
    
}