package frc.robot.commands.robot.collectalgaereef;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ReefLevel;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.CommandConstants;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;
import frc.robot.util.ReefFaceInfo;
import frc.robot.util.ReefUtil;


public class CollectAlgaeReefScoopCmd extends XeroSequenceCmd {
    private BrainSubsystem brain_ ;
    private ManipulatorSubsystem manipulator_;
    private GrabberSubsystem grabber_;
    private ReefLevel height_ ;
    private Drive db_ ;
    private boolean collect_ ;
    private boolean quick_ ;

    public CollectAlgaeReefScoopCmd(BrainSubsystem brain, Drive db, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, ReefLevel height, boolean collect, boolean quick) {
        super("CollectAlgaeReefScoopCmd") ;
        brain_ = brain ;
        db_ = db ;
        manipulator_ = manipulator;
        grabber_ = grabber;
        height_ = height ;
        collect_ = collect ;
        quick_ = quick ;
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
        // This is about the actual collect operation
        //
        Pose2d backupPose = reefFace.get().getAlgaeBackupPose() ;
        
        seq.addCommands(
            new GoToCmdDirect(manipulator_, height, angle),
            RobotContainer.getInstance().gamepad().setLockCommand(true),
            collect_ ? grabber_.setVoltageCommand(Volts.of(-6.0)) : Commands.none(),
            DriveCommands.simplePathCommand(db_, reefFace.get().getAlgaeCollectPose(), backupPose,
                                            MetersPerSecond.of(2.0), 
                                            CommandConstants.ReefDrive.kMaxDriveAcceleration),
            new GoToCmdDirect(
                manipulator_,
                height.plus(Meters.of(0.05)),
                angle.minus(Degrees.of(40))
            ).raceWith(
                Commands.waitTime(Milliseconds.of(300))
                    .finallyDo(() -> System.out.println("Removal Deadline Hit"))
            ),
            DriveCommands.simplePathCommand(db_, backupPose, backupPose, 
                                            MetersPerSecond.of(3.0), 
                                            MetersPerSecondPerSecond.of(3.0)));

        Command postseq ;

        if (!quick_) {
            postseq = Commands.sequence(                        
                RobotContainer.getInstance().gamepad().setLockCommand(false),
                new SetHoldingCmd(brain_, GamePiece.ALGAE_HIGH),
                new GoToCmdDirect(manipulator_, manipulator_.getElevatorPosition(), 
                                            ManipulatorConstants.Arm.Positions.kAlgaeReefHold),
                new GentleLowerElevator(manipulator_, ManipulatorConstants.Elevator.Positions.kAlgaeReefHold)) ;
        }
        else {
            postseq = RobotContainer.getInstance().gamepad().setLockCommand(false)
                .alongWith(collect_ ? new SetHoldingCmd(brain_, GamePiece.ALGAE_HIGH) : Commands.none());
        }
        
        
        seq.addCommands(
            new ConditionalCommand(
                postseq,
                Commands.sequence(
                    RobotContainer.getInstance().gamepad().setLockCommand(false),                        
                    grabber_.setVoltageCommand(Volts.zero()),
                    new GoToCmdDirect(manipulator_, height, angle)
                ),
                grabber_::hasAlgae));
        seq.addCommands(RobotContainer.getInstance().gamepad().setLockCommand(false)) ;
    }
}
