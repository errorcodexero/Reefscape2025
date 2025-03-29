package frc.robot.commands.robot.collectalgaereef;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ReefLevel;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.CommandConstants;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.CollectAlgaeNewCmd;
import frc.robot.subsystems.grabber.commands.RunGrabberVoltsCmd;
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
    private boolean eject_ ;

    public CollectAlgaeReefCmd(BrainSubsystem brain, Drive db, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, ReefLevel height, boolean eject) {
        super("CollectAlgaeReefCmd") ;
        brain_ = brain ;
        db_ = db ;
        manipulator_ = manipulator;
        grabber_ = grabber;
        height_ = height ;
        eject_ = eject ;
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

        Alliance a = DriverStation.getAlliance().get() ;

        //
        // This is about the actual collect operation
        //
        Pose2d bup = reefFace.get().getAlgaeBackupPose() ;
        Pose2d buprot ; 
        
        if (eject_) {
            buprot = new Pose2d(bup.getTranslation(), (a == Alliance.Red) ? Rotation2d.fromDegrees(180.0) : Rotation2d.fromDegrees(0.0)) ;
        }
        else {
            buprot = bup ;
        }
        
        seq.addCommands(
            new GoToCmdDirect(manipulator_, height, angle),
            RobotContainer.getInstance().gamepad().setLockCommand(true),
            grabber_.setVoltageCommand(Volts.of(-6.0)),
            DriveCommands.simplePathCommand(db_, reefFace.get().getAlgaeCollectPose(), bup,
                                            MetersPerSecond.of(2.0), 
                                            CommandConstants.ReefDrive.kMaxDriveAcceleration),
            new CollectAlgaeNewCmd(grabber_, manipulator_, level),
            DriveCommands.simplePathCommand(db_, buprot, bup, 
                                            MetersPerSecond.of(3.0), 
                                            MetersPerSecondPerSecond.of(3.0))) ;
        if (eject_) {
            seq.addCommands(new RunGrabberVoltsCmd(grabber_, Milliseconds.of(500))) ;
        }
        else {
            seq.addCommands(
                new ConditionalCommand(
                    Commands.sequence(
                        RobotContainer.getInstance().gamepad().setLockCommand(false),
                        new SetHoldingCmd(brain_, GamePiece.ALGAE_HIGH),
                        new GoToCmdDirect(manipulator_, manipulator_.getElevatorPosition(), 
                                                        ManipulatorConstants.Arm.Positions.kAlgaeReefHold),
                        new GentleLowerElevator(manipulator_, ManipulatorConstants.Elevator.Positions.kAlgaeReefHold)
                    ),
                    Commands.sequence(
                        RobotContainer.getInstance().gamepad().setLockCommand(false),                        
                        grabber_.setVoltageCommand(Volts.zero()),
                        new GoToCmdDirect(manipulator_, height, angle)
                    ),
                    this::hasAlgae)) ;
        }
        seq.addCommands(RobotContainer.getInstance().gamepad().setLockCommand(false)) ;
    }

    private boolean hasAlgae() {
        return grabber_.hasAlgae() ;
    }
}
