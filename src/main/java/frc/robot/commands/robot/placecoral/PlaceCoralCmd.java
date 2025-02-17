package frc.robot.commands.robot.placecoral;

import java.util.Optional;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ReefLevel;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.CommandConstants;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Arm;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Elevator;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.util.ReefUtil;
import frc.robot.util.ReefUtil.ReefFace;

public class PlaceCoralCmd extends Command {

    private XeroSequence sequence_;

    private final Drive drive_;
    private final ManipulatorSubsystem manipulator_; 
    private final GrabberSubsystem grabber_; 
    private final BrainSubsystem brain_; 

    private final CoralSide side_ ;
    private final ReefLevel level_ ;

    private Distance target_elev_pos_; 
    private Angle target_arm_pos_; 
    private boolean driveto_ ;

    public PlaceCoralCmd(BrainSubsystem brain, Drive drive, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, boolean driveto, ReefLevel h, CoralSide s) {
        drive_ = drive;
        manipulator_ = manipulator; 
        grabber_ = grabber;
        brain_ = brain; 

        side_ = s ;
        level_ = h ;

        target_elev_pos_ = Elevator.Positions.kStow; 
        target_arm_pos_ = Arm.Positions.kStow; 

        driveto_ = driveto ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ReefLevel level ;
        CoralSide side ;

        sequence_ = new XeroSequence();

        if (level_ == ReefLevel.AskBrain) {
            level = brain_.coralLevel() ;
        }
        else {
            level = level_ ;
        }

        if (side_ == CoralSide.AskBrain) {
            side = brain_.coralSide() ;
        }
        else {
            side = side_ ;
        }

        if (DriverStation.getAlliance().isEmpty())
            return ;

        Optional<ReefFace> reefFace = ReefUtil.getTargetedReefFace(drive_.getPose());
        if (reefFace.isEmpty())
            return ;

        if (level == ReefLevel.AskBrain || side == CoralSide.AskBrain) {
            //
            // Should never happen, but if it does, we just return and do nothing
            //
            return ;
        }

        switch(level) {
            case L1:
                target_elev_pos_ = Elevator.Positions.kPlaceL1;
                target_arm_pos_ = Arm.Positions.kPlaceL1;  
                break ;

            case L2:
                target_elev_pos_ = Elevator.Positions.kPlaceL2; 
                target_arm_pos_ = Arm.Positions.kPlaceL2;  
                break ;
            
            case L3:
                target_elev_pos_ = Elevator.Positions.kPlaceL3; 
                target_arm_pos_ = Arm.Positions.kPlaceL3;  
                break ;

            case L4:
                target_elev_pos_ = Elevator.Positions.kPlaceL4;
                target_arm_pos_ = Arm.Positions.kPlaceL4;
                break ;

            default:
                // Just to keep the intellisense happy
                break ;
        }

        ReefFace face = reefFace.get();
        Pose2d scoringPose = side == CoralSide.Left ? face.getLeftScoringPose() : face.getRightScoringPose();

        if (driveto_) {
            sequence_.addCommands(
                RobotContainer.getInstance().gamepad().setLockCommand(true),
                Commands.parallel(
                    new GoToCmd(manipulator_, target_elev_pos_, target_arm_pos_),
                    DriveCommands.simplePathCommand(drive_, scoringPose, CommandConstants.ReefDrive.kMaxDriveVelocity, CommandConstants.ReefDrive.kMaxDriveAcceleration))) ;
        }
        else {
            sequence_.addCommands(
                new GoToCmd(manipulator_, target_elev_pos_, target_arm_pos_)) ; 
        }

        sequence_.addCommands(
            new GoToCmd(manipulator_, target_elev_pos_, target_arm_pos_, true),
            new DepositCoralCmd(grabber_),
            new SetHoldingCmd(brain_, GamePiece.NONE),
            new GoToCmd(manipulator_, target_elev_pos_, ManipulatorConstants.Arm.Positions.kKickbackAngle, true),
            new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;

        if (driveto_) {
            sequence_.addCommands(
                RobotContainer.getInstance().gamepad().setLockCommand(false)) ;
        }
        sequence_.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        sequence_.cancel(); 
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return sequence_.isComplete(); 
    }
}
