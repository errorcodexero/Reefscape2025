package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;

public class CollectingCoralCmd extends Command {
  private final FunnelSubsystem funnel_;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CollectingCoralCmd(FunnelSubsystem funnel) {
    funnel_ = funnel;

    addRequirements(funnel_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return funnel_.hasSeenCoral();
  }
}
