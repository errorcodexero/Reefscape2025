package frc.robot.commands.robot.placecoral;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;

import java.util.Optional;

import org.xerosw.util.XeroSequenceCmd;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
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
import frc.robot.subsystems.manipulator.ManipulatorConstants.Arm;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Elevator;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;
import frc.robot.subsystems.manipulator.commands.GoToWhenClose;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.util.ReefFaceInfo;
import frc.robot.util.ReefUtil;

public class PlaceCoralCmd extends XeroSequenceCmd {

    private static final Distance kRaiseElevatorDistance = Centimeters.of(50.0) ;

    private final Drive drive_;
    private final ManipulatorSubsystem manipulator_; 
    private final GrabberSubsystem grabber_; 
    private final BrainSubsystem brain_; 

    private final CoralSide side_ ;
    private final ReefLevel level_ ;

    private Distance target_elev_pos_; 
    

    private boolean lower_manip_ ;

    private Distance raiseDistance() {
        if (RobotState.isAutonomous()) {
            return Centimeters.of(75.0) ;
        }

        return kRaiseElevatorDistance ;
    }

    public PlaceCoralCmd(BrainSubsystem brain, Drive drive, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, ReefLevel h, CoralSide s) {
        this(brain, drive, manipulator, grabber, h, s, true) ;
    }

    public PlaceCoralCmd(BrainSubsystem brain, Drive drive, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, ReefLevel h, CoralSide s, boolean lower) {
        super("PlaceCoralCmd") ;
        drive_ = drive;
        manipulator_ = manipulator; 
        grabber_ = grabber;
        brain_ = brain; 

        side_ = s ;
        level_ = h ;

        target_elev_pos_ = Elevator.Positions.kStow; 

        lower_manip_ = lower ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initSequence(SequentialCommandGroup seq) {
        ReefLevel level ;
        CoralSide side ;
        Angle immdangle ;

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

        immdangle = Arm.Positions.kRaiseAngle ;
        switch(level) {
            case L1:
                target_elev_pos_ = Elevator.Positions.kPlaceL1;
                break ;

            case L2:
                target_elev_pos_ = Elevator.Positions.kPlaceL2; 
                immdangle = Arm.Positions.kPlaceL2 ;
                break ;
            
            case L3:
                target_elev_pos_ = Elevator.Positions.kPlaceL3; 
                immdangle = Arm.Positions.kPlaceL3 ;
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

        seq.addCommands(RobotContainer.getInstance().gamepad().setLockCommand(true)) ;

        if (level == ReefLevel.L4) {
            seq.addCommands(
                Commands.parallel(
                    DriveCommands.simplePathCommand(drive_, scoringPose, maxvel, maxaccel),
                    new GoToWhenClose(drive_, manipulator_, 
                                    target_elev_pos_, Centimeters.of(1.0), MetersPerSecond.of(500.0),
                                    immdangle, Degrees.of(3.0), DegreesPerSecond.of(5.0), 
                                    scoringPose, raiseDistance())
                ),
                new PositionToPlaceCmd(drive_, brain_, manipulator_, grabber_, level, scoringPose),
                Commands.parallel(
                    new DepositCoralCmd(grabber_, brain_.coralLevel()),
                    Commands.sequence(
                        new WaitCommand(Milliseconds.of(100)),
                        new GoToCmdDirect(manipulator_, target_elev_pos_, Centimeters.of(18.0), MetersPerSecond.of(500.0), 
                                                        ManipulatorConstants.Arm.Positions.kKickbackAngle, Degrees.of(5.0), DegreesPerSecond.of(10.0))
                    )
                )) ;
        }
        else {
            seq.addCommands(
                Commands.parallel(
                    DriveCommands.simplePathCommand(drive_, scoringPose, maxvel, maxaccel),
                    new GoToWhenClose(drive_, manipulator_, target_elev_pos_, Centimeters.of(3.0), MetersPerSecond.of(0.01),
                    immdangle, Degrees.of(3.0), DegreesPerSecond.of(5.0), scoringPose, raiseDistance())
                ),
                new DepositCoralCmd(grabber_, brain_.coralLevel())
            ) ;            
        }

        seq.addCommands(
            Commands.runOnce(()-> brain_.setGoingDown(true)),
            new SetHoldingCmd(brain_, GamePiece.NONE)
        ) ;
            

        if (lower_manip_) {
            seq.addCommands(new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;
        }

        seq.addCommands(RobotContainer.getInstance().gamepad().setLockCommand(false)) ;
    }
}
