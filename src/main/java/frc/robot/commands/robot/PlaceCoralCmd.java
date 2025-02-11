package frc.robot.commands.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.util.ReefUtil;
import frc.robot.util.ReefUtil.ReefFace;

public class PlaceCoralCmd extends Command {

  private final XeroSequence sequence_;

  private final Drive drive_;
  private final ManipulatorSubsystem manipulator_; 
  private final GrabberSubsystem grabber_; 

  public PlaceCoralCmd(Drive drive, ManipulatorSubsystem manipulator, GrabberSubsystem grabber) {
    addRequirements(drive, manipulator, grabber);

    sequence_ = new XeroSequence();

    drive_ = drive;
    manipulator_ = manipulator; 
    grabber_ = grabber;
  }

  // ACTIONS NEEDED: 
    // drive to cmd
    // go to elevator
    // go to arm
    // deposit coral
    // go to arm (slightly angle up from branch)
    // stow elevator
    // rumble gamepad

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // TO-DO: 
    // use brain subsystem to get what level and side we need to score on,
    // the elevator and arm values for the methods that move them are currently null

    Optional<ReefFace> reefFace = ReefUtil.getTargetedReefFace(drive_.getPose());
    // TO-DO: check if this is not there
    ReefFace face = reefFace.get();
    boolean left = true;
    Pose2d scoringPose = left ? face.getLeftScoringPose() : face.getRightScoringPose();
    // TO-DO: change to other path following method

    Command drive = DriveCommands.swerveDriveToCommand(scoringPose);
    GoToCmd goToPlaceElevator = new GoToCmd(manipulator_, null, null);
    GoToCmd goToPlaceArm = new GoToCmd(manipulator_, null, null); 
    DepositCoralCmd depositCoral = new DepositCoralCmd(grabber_);
    GoToCmd moveArmBack = new GoToCmd(manipulator_, null, ManipulatorConstants.Arm.Positions.kKickbackAngle);
    GoToCmd stowElevator = new GoToCmd(manipulator_, Meters.of(0), null); 

    sequence_.addCommands(drive, goToPlaceElevator, goToPlaceArm, depositCoral, moveArmBack, stowElevator);
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
