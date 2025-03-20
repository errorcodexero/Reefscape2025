package frc.robot.commands.robot.collectalgaereef;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ReefLevel;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.misc.StateCmd;
import frc.robot.commands.robot.CommandConstants;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.CollectAlgaeNewCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;
import frc.robot.util.ReefFaceInfo;
import frc.robot.util.ReefUtil;

public class CollectAlgaeReefCmd extends XeroSequenceCmd {
    private BrainSubsystem brain_ ;
    private ManipulatorSubsystem manipulator_;
    private GrabberSubsystem grabber_;
    private ReefLevel height_ ;
    private Drive db_ ;
    private boolean skipfirst_ ;

    public CollectAlgaeReefCmd(BrainSubsystem brain, Drive db, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, ReefLevel height) {
        this(brain, db, manipulator, grabber, height, false) ;
    }

    public CollectAlgaeReefCmd(BrainSubsystem brain, Drive db, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, ReefLevel height, boolean skipfirst) {
        super("CollectAlgaeReefCmd") ;
        brain_ = brain ;
        db_ = db ;
        manipulator_ = manipulator;
        grabber_ = grabber;
        height_ = height ;
        skipfirst_ = skipfirst ;
    }

    // COMMANDS NEEDED:
    // GoToCmd
    // WaitForCoral

    private boolean alreadyRotated() {
        return manipulator_.getArmPosition().gt(ManipulatorConstants.Arm.Positions.kFinishedAlgaeThreshhold) ;
    }

    @Override
    public void initSequence(SequentialCommandGroup seq) {
        ReefLevel level = height_ ;

        Angle angle ;
        Distance height ;
        
        if (height_ == ReefLevel.AskBrain) {
            level = brain_.algaeLevel() ;
        }

        if (level == ReefLevel.L2 || level == ReefLevel.L1) {
            angle = ManipulatorConstants.Arm.Positions.kAlgaeReefCollectNewL2 ;
            height = ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectNewL2 ;
        } else if (level == ReefLevel.L3 || level == ReefLevel.L4) {
            angle = ManipulatorConstants.Arm.Positions.kAlgaeReefCollectNewL3 ;
            height = ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectNewL3 ;
        }
        else {
            //
            // Bad height value, so we just return and have an empty sequence which
            // does nothing.
            //
            return ;
        }

        Optional<ReefFaceInfo> reefFace = ReefUtil.getTargetedReefFace(db_.getPose());
        if (reefFace.isEmpty())
            return ;

        //
        // This is all about getting the arm and elevator to the right place with the least amount of motion
        //
        if (!skipfirst_) {
            seq.addCommands(
                db_.stopCmd(),
                new ConditionalCommand(
                    // If we are already rotated to the correct angle, skip the sequence to the raise angle
                    new GoToCmdDirect(manipulator_, height, angle),

                    // Otherwise carefully go through the correct sequence
                    Commands.sequence(
                        new GoToCmdDirect(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, manipulator_.getArmPosition()),
                        new GoToCmdDirect(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, angle),
                        new GoToCmdDirect(manipulator_, height, angle)
                    ),
                    this::alreadyRotated)) ;
        }
        else {
            seq.addCommands(new GoToCmdDirect(manipulator_, height, angle)) ;
        }

        //
        // This is about the actual collect operation
        //
        
        seq.addCommands(
            new StateCmd("algaecollect", "newversion"),
            RobotContainer.getInstance().gamepad().setLockCommand(true),
            grabber_.setVoltageCommand(Volts.of(-6.0)),
            DriveCommands.simplePathCommand(db_, reefFace.get().getAlgaeCollectPose(),
                                            MetersPerSecond.of(1.0), 
                                            CommandConstants.ReefDrive.kMaxDriveAcceleration));

        seq.addCommands(
            new CollectAlgaeNewCmd(grabber_, manipulator_, level)
        );

        seq.addCommands(
            DriveCommands.simplePathCommand(db_, reefFace.get().getAlgaeBackupPose(), 
                                            MetersPerSecond.of(2.0), 
                                            MetersPerSecondPerSecond.of(2.0)),
            new ConditionalCommand(
                new SetHoldingCmd(brain_, GamePiece.ALGAE_HIGH),
                Commands.none(),
                this::hasAlgae),
            RobotContainer.getInstance().gamepad().setLockCommand(false),
            new GoToCmdDirect(manipulator_, ManipulatorConstants.Elevator.Positions.kAlgaeReefHold, 
                                      ManipulatorConstants.Arm.Positions.kAlgaeReefHold)) ;
    }

    private boolean hasAlgae() {
        return !grabber_.algaeSensor() ;
    }
}
