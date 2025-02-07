package frc.robot.subsystems.manipulator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

public class GoToCmd extends Command {
    private Distance target_height_; 
    private Angle target_angle_;
    private ManipulatorSubsystem sub_;
    private State state_; 

    private enum State {
      GoToFinalPos, 
      WaitForArm,
      WaitForElevator, 
      Done
    }

  public GoToCmd(ManipulatorSubsystem sub, Distance targetElevPos, Angle targetArmPos) {
    addRequirements(sub); 
      
    sub_ = sub;
    target_height_ = targetElevPos; 
    target_angle_ = targetArmPos; 
  }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Distance current_height = sub_.getElevatorPosition();
    Angle current_angle = sub_.getArmPosition();

    if((current_height.gt(ManipulatorConstants.Keepout.kKeepoutHeight)) && (target_height_.gt(ManipulatorConstants.Keepout.kKeepoutHeight))) {
      sub_.setElevatorPosition(target_height_);
      sub_.setArmPosition(target_angle_);

      state_ = State.GoToFinalPos;  

    } else if(current_height.gt(ManipulatorConstants.Keepout.kKeepoutHeight) && target_height_.lte(ManipulatorConstants.Keepout.kKeepoutHeight)) {
      sub_.setElevatorPosition(ManipulatorConstants.Keepout.kKeepoutHeight);
      sub_.setArmPosition(target_angle_);

      state_ = State.WaitForArm; 

    } else if(current_height.lte(ManipulatorConstants.Keepout.kKeepoutHeight) && !sub_.doesCrossKZ(current_angle, target_angle_)) {
      sub_.setElevatorPosition(target_height_);
      sub_.setArmPosition(target_angle_);

      state_ = State.GoToFinalPos; 

    } else if(current_height.lte(ManipulatorConstants.Keepout.kKeepoutHeight) && sub_.doesCrossKZ(current_angle, target_angle_)) {
      sub_.setElevatorPosition(ManipulatorConstants.Keepout.kKeepoutHeight); 
      
      state_ = State.WaitForElevator; 
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state_) {
      case GoToFinalPos:
        if(sub_.isElevAtTarget() && sub_.isArmAtTarget()) {
          state_ = State.Done; 
        }
        break;

      case WaitForArm:
        if(sub_.isArmAtTarget()) {
          sub_.setElevatorPosition(target_height_);
        }
        
        if(sub_.isElevAtTarget()) {
          state_ = State.Done; 
        }
        break;

      case WaitForElevator:
        if(sub_.isElevAtTarget()) {
          if(target_height_.gt(ManipulatorConstants.Keepout.kKeepoutHeight)) {
            sub_.setElevatorPosition(target_height_);
            sub_.setArmPosition(target_angle_);

            state_ = State.GoToFinalPos; 
          } else {
            sub_.setArmPosition(target_angle_); 

            state_ = State.WaitForArm; 
          }
        }
        break;

      case Done: 
        break; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state_ == State.Done; 
  }
}
