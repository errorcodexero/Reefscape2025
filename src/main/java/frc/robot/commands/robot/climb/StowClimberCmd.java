package frc.robot.commands.robot.climb;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberPositionCmd;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.funnel.DeployFunnelCmd;
import frc.robot.subsystems.funnel.FunnelSubsystem;

public class StowClimberCmd extends XeroSequenceCmd {
    private ClimberSubsystem climber_ ;
    private FunnelSubsystem funnel_ ;

    public StowClimberCmd(ClimberSubsystem climber, FunnelSubsystem funnel) {
        super("StowClimberCmd");
        climber_ = climber ;
        funnel_ = funnel ;
    }

    @Override
    public void initSequence(SequentialCommandGroup sequence) {
        sequence.addCommands(new ClimberPositionCmd(climber_, ClimberState.Stowed)) ;
        sequence.addCommands(new DeployFunnelCmd(funnel_, ClimbConstants.Funnel.kNormalAngle)) ;   
    }
}
