package frc.robot.commands.robot.placecoral;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Milliseconds;

import java.util.Optional;

import org.xerosw.util.XeroSequenceCmd;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ReefLevel;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.CommandConstants;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositCoralCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Elevator;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;
import frc.robot.subsystems.manipulator.commands.GoToWhenClose;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.util.ReefFaceInfo;
import frc.robot.util.ReefUtil;

public class PlaceCoralCmd extends XeroSequenceCmd {

    private static final Distance kRaiseElevatorDistance = Centimeters.of(30.0) ;

    private final Drive drive_;
    private final ManipulatorSubsystem manipulator_; 
    private final GrabberSubsystem grabber_; 
    private final BrainSubsystem brain_; 

    private final CoralSide side_ ;
    private final ReefLevel level_ ;

    private Distance target_elev_pos_; 
    

    private boolean lower_manip_ ;
    private boolean drive_while_raising_ ;

    public PlaceCoralCmd(BrainSubsystem brain, Drive drive, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, ReefLevel h, CoralSide s) {
        this(brain, drive, manipulator, grabber, h, s, true, false) ;
    }

    public PlaceCoralCmd(BrainSubsystem brain, Drive drive, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, ReefLevel h, CoralSide s, boolean lower, boolean drive_while_raising) {
        super("PlaceCoralCmd") ;
        drive_ = drive;
        manipulator_ = manipulator; 
        grabber_ = grabber;
        brain_ = brain; 

        side_ = s ;
        level_ = h ;

        target_elev_pos_ = Elevator.Positions.kStow; 

        lower_manip_ = lower ;
        drive_while_raising_ = drive_while_raising ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initSequence(SequentialCommandGroup seq) {
        ReefLevel level ;
        CoralSide side ;

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

        brain_.setPlacedOk(true);

        if (DriverStation.getAlliance().isEmpty()) {
            brain_.setPlacedOk(false);
            return ;
        }

        Pose2d robotPose = drive_.getPose() ;
        Optional<ReefFaceInfo> reefFace = ReefUtil.getTargetedReefFace(robotPose) ;
        if (reefFace.isEmpty()) {
            brain_.setPlacedOk(false);
            return ;
        }

        if (level == ReefLevel.AskBrain || side == CoralSide.AskBrain) {
            //
            // Should never happen, but if it does, we just return and do nothing
            //
            return ;
        }

        LinearVelocity maxvel = CommandConstants.ReefDrive.kMaxDriveVelocity ;
        LinearAcceleration maxaccel = CommandConstants.ReefDrive.kMaxDriveAcceleration ;

        switch(level) {
            case L1:
                target_elev_pos_ = Elevator.Positions.kPlaceL1;
                break ;

            case L2:
                target_elev_pos_ = Elevator.Positions.kPlaceL2; 
                break ;
            
            case L3:
                target_elev_pos_ = Elevator.Positions.kPlaceL3; 
                break ;

            case L4:
                target_elev_pos_ = Elevator.Positions.kPlaceL4;
                break ;

            default:
                // Just to keep the intellisense happy
                break ;
        }

        ReefFaceInfo face = reefFace.get();
        Pose2d scoringPose ;
        if (brain_.doesReefHaveAlgae()) {
            scoringPose = side == CoralSide.Left ? face.getLeftScoringWithAlgaePose() : face.getRightScoringWithAlgaePose();
        }
        else {
            scoringPose = side == CoralSide.Left ? face.getLeftScoringPose() : face.getRightScoringPose();            
        }


        seq.addCommands(
            RobotContainer.getInstance().gamepad().setLockCommand(true)) ;

        seq.addCommands(
            Commands.parallel(
                DriveCommands.simplePathCommand(drive_, scoringPose, maxvel, maxaccel),
                new GoToWhenClose(drive_, manipulator_, target_elev_pos_, 
                    ManipulatorConstants.Arm.Positions.kRaiseAngle, scoringPose, kRaiseElevatorDistance)
            )) ;
        seq.addCommands(
            new PositionToPlaceCmd(drive_, brain_, manipulator_, grabber_, level, scoringPose),
            Commands.parallel(
                new DepositCoralCmd(grabber_, brain_.coralLevel()),
                Commands.sequence(
                    new WaitCommand(Milliseconds.of(100)),
                    new GoToCmdDirect(manipulator_, target_elev_pos_, ManipulatorConstants.Arm.Positions.kKickbackAngle)
                )
            ),

            Commands.runOnce(()-> brain_.setGoingDown(true)),
            new SetHoldingCmd(brain_, GamePiece.NONE)
        ) ;
            

        if (lower_manip_) {
            seq.addCommands(new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;
        }

        seq.addCommands(RobotContainer.getInstance().gamepad().setLockCommand(false)) ;
    }

    private boolean raiseWhileDriving() {
        boolean ret = false ;
        if (drive_while_raising_) {
            if (level_ == ReefLevel.AskBrain) {
                ReefLevel l = brain_.coralLevel() ;
                if (l != ReefLevel.L4) {
                    ret = true ;
                }
            }
            else {
                if (level_ != ReefLevel.L4 || RobotState.isAutonomous()) {
                    ret = true ;
                }
            }
        }
        ret = true ;
        return ret;
    }
}
