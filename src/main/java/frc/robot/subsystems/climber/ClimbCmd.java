package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCmd extends Command {
    private ClimberSubsystem climber_;
    private boolean holding_ ;

    public ClimbCmd(ClimberSubsystem climber) {
        addRequirements(climber);

        climber_ = climber;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climber_.setMotorVoltage(Volts.of(ClimberConstants.Climber.kClimbVoltage));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Logger.recordOutput("climber/holding", holding_) ;

        if (!holding_) {
            if (climber_.getClimberPosition().isNear(ClimberConstants.Climber.Position.kClimbed, ClimberConstants.Climber.kPosTolerance)) {
                climber_.setClimberTarget(ClimberState.Climb) ;
                holding_ = true ;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
