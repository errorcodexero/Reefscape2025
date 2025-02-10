package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberPositionCmd.ClimberTarget;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.funnel.DeployFunnelCmd;
import frc.robot.subsystems.funnel.FunnelConstants;


public class PrepClimbCmd extends SequentialCommandGroup {
    public PrepClimbCmd(ClimberSubsystem climberSubsystem) {
        addCommands(
            new ClimberPositionCmd(climberSubsystem, ClimberTarget.PrepareToClimb)
        );
    }

}

