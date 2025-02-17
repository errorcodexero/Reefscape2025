package frc.robot.commands.robot.collectalgaereef;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.Optional;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.ReefLevel;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.CommandConstants;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.CollectAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.util.ReefUtil;
import frc.robot.util.ReefUtil.ReefFace;

public class CollectAlgaeReefCmd extends Command {
    private XeroSequence sequence_;
    private BrainSubsystem brain_ ;
    private ManipulatorSubsystem manipulator_;
    private GrabberSubsystem grabber_;
    private ReefLevel height_ ;
    private Drive db_ ;
    private boolean driveto_ ;

    public CollectAlgaeReefCmd(BrainSubsystem brain, Drive db, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, ReefLevel height) {
        this(brain, db, manipulator, grabber, height, true) ;
    }

    public CollectAlgaeReefCmd(BrainSubsystem brain, Drive db, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, ReefLevel height, boolean driveto) {
        brain_ = brain ;
        db_ = db ;
        manipulator_ = manipulator;
        grabber_ = grabber;
        height_ = height ;
        driveto_ = driveto ;
    }

    // COMMANDS NEEDED:
    // GoToCmd
    // WaitForCoral

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ReefLevel level = height_ ;

        Angle angle ;
        Distance height ;
        
        if (height_ == ReefLevel.AskBrain) {
            level = brain_.algaeLevel() ;
        }

        if (level == ReefLevel.L2 || level == ReefLevel.L1) {
            angle = ManipulatorConstants.Arm.Positions.kAlgaeReefCollectL2 ;
            height = ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL2 ;
        } else if (level == ReefLevel.L3 || level == ReefLevel.L4) {
            angle = ManipulatorConstants.Arm.Positions.kAlgaeReefCollectL3 ;
            height = ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL3 ;
        }
        else {
            //
            // Bad height value, so we just return and have an empty sequence which
            // does nothing.
            //
            return ;
        }

        Optional<ReefFace> reefFace = ReefUtil.getTargetedReefFace(db_.getPose());
        if (reefFace.isEmpty())
            return ;

        sequence_ = new XeroSequence();

        sequence_.addCommands(
            new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL3, 
                                      ManipulatorConstants.Arm.Positions.kRaiseAngle),
            new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL3, angle, true),
            new GoToCmd(manipulator_, height, angle, true)) ;
        
        if (driveto_) {
            sequence_.addCommands(
                RobotContainer.getInstance().gamepad().setLockCommand(true),
                Commands.parallel(
                    DriveCommands.simplePathCommand(db_, reefFace.get().getAlgaeCollectPose(), 
                                                    MetersPerSecond.of(1.0), 
                                                    CommandConstants.ReefDrive.kMaxDriveAcceleration),
                    new CollectAlgaeCmd(grabber_))) ;
        }
        else {
            sequence_.addCommands(
               new CollectAlgaeCmd(grabber_)) ;
        }
        sequence_.addCommands(
            new SetHoldingCmd(brain_, GamePiece.ALGAE_HIGH),
            DriveCommands.simplePathCommand(db_, reefFace.get().getAlgaeBackupPose(), 
                                            MetersPerSecond.of(1.0), 
                                            MetersPerSecondPerSecond.of(1.0)),
            RobotContainer.getInstance().gamepad().setLockCommand(false),
            new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kAlgaeReefHold, 
                                      ManipulatorConstants.Arm.Positions.kAlgaeReefHold, true)) ;

        sequence_.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            sequence_.cancel();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return sequence_.isComplete();
    }
}
