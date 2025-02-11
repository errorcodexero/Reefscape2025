package frc.robot.commands.robot;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class StowArmElevatorCmd extends Command {

  private XeroSequence sequence_; 
  private ManipulatorSubsystem manipulator_; 

  public StowArmElevatorCmd(ManipulatorSubsystem manipulator) {
    addRequirements(manipulator); 

    manipulator_ = manipulator; 
  }

  // COMMANDS NEEDED: 
  // GoToCmd

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GoToCmd goToCmd = new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow);
    sequence_.addCommands(goToCmd);
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
