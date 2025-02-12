package frc.robot.commands.robot.placecoral;

import java.util.Optional;
import org.xerosw.util.XeroSequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Height;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Elevator;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Arm;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.util.ReefUtil;
import frc.robot.util.ReefUtil.ReefFace;

public class PlaceCoralCmd extends Command {

    private final XeroSequence sequence_;

    private final Drive drive_;
    private final ManipulatorSubsystem manipulator_; 
    private final GrabberSubsystem grabber_; 
    private final BrainSubsystem brain_; 

    private final CoralSide side_ ;
    private final Height level_ ;

    private Distance target_elev_pos_; 
    private Angle target_arm_pos_; 

    public PlaceCoralCmd(Drive drive, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, BrainSubsystem brain, boolean driveto, Height h, CoralSide s) {
        addRequirements(drive, manipulator, grabber, brain);

        sequence_ = new XeroSequence();

        drive_ = drive;
        manipulator_ = manipulator; 
        grabber_ = grabber;
        brain_ = brain; 

        side_ = s ;
        level_ = h ;

        target_elev_pos_ = Elevator.Positions.kStow; 
        target_arm_pos_ = Arm.Positions.kStow; 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Height level ;
        CoralSide side ;

        if (level_ == Height.AskBrain) {
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

        if (level == Height.AskBrain || side == CoralSide.AskBrain) {
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

        sequence_.addCommands(
            DriveCommands.simplePathCommand(scoringPose),
            new GoToCmd(manipulator_, target_elev_pos_, target_arm_pos_),
            new DepositCoralCmd(grabber_),
            new GoToCmd(manipulator_, target_elev_pos_, ManipulatorConstants.Arm.Positions.kKickbackAngle),
            new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;

        sequence_.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

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
