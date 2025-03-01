package frc.robot.commands.robot.climb;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberPositionCmd;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.subsystems.funnel.DeployFunnelCmd ;

public class PrepClimbCmd extends XeroSequenceCmd {
    private ClimberSubsystem climber_ ;
    private FunnelSubsystem funnel_ ;
    private ManipulatorSubsystem m_ ;

    public PrepClimbCmd(ClimberSubsystem climber, FunnelSubsystem funnel, ManipulatorSubsystem manipulator) {
        super("PrepClimbCmd");

        climber_ = climber ;
        funnel_ = funnel ;
        m_ = manipulator ;
    }

    @Override
    public void initSequence(SequentialCommandGroup sequence) {
        sequence.addCommands(new DeployFunnelCmd(funnel_, DeployFunnelCmd.Position.Climb)) ;
        sequence.addCommands(new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kClimb)) ;
        sequence.addCommands(new ClimberPositionCmd(climber_, ClimberState.PrepareToClimb)) ;
    }
}
