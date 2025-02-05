package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;

public class PrepClimbCmd extends Command{
    private ClimberSubsystem climber_;
    private PrepClimbCmdState PrepClimbCmdState_;

    private enum PrepClimbCmdState{
        RetractCoralCollector,
        DeployClimber,
        ClimberDeployed,
        Done
    }

}

