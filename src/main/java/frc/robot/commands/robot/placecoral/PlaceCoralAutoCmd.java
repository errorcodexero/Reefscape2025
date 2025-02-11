package frc.robot.commands.robot.placecoral;

import static edu.wpi.first.units.Units.*;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Elevator;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Arm;

public class PlaceCoralAutoCmd extends Command {

  private final XeroSequence sequence_;

  private final ManipulatorSubsystem manipulator_; 
  private final GrabberSubsystem grabber_; 

  private Distance target_elev_pos_; 
  private Angle target_arm_pos_; 
  private int coral_level_; 

  public PlaceCoralAutoCmd(ManipulatorSubsystem manipulator, GrabberSubsystem grabber, BrainSubsystem brain, int coralLevel) {
    addRequirements(manipulator, grabber, brain);

    sequence_ = new XeroSequence();

    manipulator_ = manipulator; 
    grabber_ = grabber;

    target_elev_pos_ = Elevator.Positions.kStow; 
    target_arm_pos_ = Arm.Positions.kStow; 
    coral_level_ = coralLevel; 
  }

  @Override
  public void initialize() {

    if(coral_level_ == 1){
      target_elev_pos_ = Elevator.Positions.kPlaceL1;
      target_arm_pos_ = Arm.Positions.kPlaceL1;  
    } else if(coral_level_ == 2){
      target_elev_pos_ = Elevator.Positions.kPlaceL2; 
      target_arm_pos_ = Arm.Positions.kPlaceL2;  
    } else if(coral_level_ == 3){
      target_elev_pos_ = Elevator.Positions.kPlaceL3; 
      target_arm_pos_ = Arm.Positions.kPlaceL3;  
    } else if(coral_level_ == 4){
      target_elev_pos_ = Elevator.Positions.kPlaceL4;
      target_arm_pos_ = Arm.Positions.kPlaceL4;   
    } 

    GoToCmd goToPlaceElevator = new GoToCmd(manipulator_, target_elev_pos_, null);
    GoToCmd goToPlaceArm = new GoToCmd(manipulator_, null, target_arm_pos_); 
    DepositCoralCmd depositCoral = new DepositCoralCmd(grabber_);
    GoToCmd moveArmBack = new GoToCmd(manipulator_, null, ManipulatorConstants.Arm.Positions.kKickbackAngle);
    GoToCmd stowElevator = new GoToCmd(manipulator_, Meters.of(0), null); 

    sequence_.addCommands(goToPlaceElevator, goToPlaceArm, depositCoral, moveArmBack, stowElevator);
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
