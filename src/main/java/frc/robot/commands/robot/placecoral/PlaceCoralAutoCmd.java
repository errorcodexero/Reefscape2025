package frc.robot.commands.robot.placecoral;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Height;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Arm;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Elevator;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class PlaceCoralAutoCmd extends Command {

  private final XeroSequence sequence_;

  private final ManipulatorSubsystem manipulator_;
  private final GrabberSubsystem grabber_;

  private Distance target_elev_pos_;
  private Angle target_arm_pos_;
  private Height coral_level_;

  public PlaceCoralAutoCmd(ManipulatorSubsystem manipulator, GrabberSubsystem grabber, Height coralLevel) {
    sequence_ = new XeroSequence();

    manipulator_ = manipulator;
    grabber_ = grabber;

    target_elev_pos_ = Elevator.Positions.kStow;
    target_arm_pos_ = Arm.Positions.kStow;
    coral_level_ = coralLevel;
  }

  @Override
  public void initialize() {
    switch (coral_level_) {
      case L1:
        target_elev_pos_ = Elevator.Positions.kPlaceL1;
        target_arm_pos_ = Arm.Positions.kPlaceL1;
        break;
      case L2:
        target_elev_pos_ = Elevator.Positions.kPlaceL2;
        target_arm_pos_ = Arm.Positions.kPlaceL2;
        break;
      case L3:
        target_elev_pos_ = Elevator.Positions.kPlaceL3;
        target_arm_pos_ = Arm.Positions.kPlaceL3;
        break;
      case L4:
        target_elev_pos_ = Elevator.Positions.kPlaceL4;
        target_arm_pos_ = Arm.Positions.kPlaceL4;
        break;
      default:
        throw new AssertionError();
    }

    sequence_.addCommands(
      new GoToCmd(manipulator_, target_elev_pos_, target_arm_pos_),
      new DepositCoralCmd(grabber_),
      new GoToCmd(manipulator_, target_elev_pos_, ManipulatorConstants.Arm.Positions.kKickbackAngle),
      new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;

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
