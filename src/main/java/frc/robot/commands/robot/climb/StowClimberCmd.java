package frc.robot.commands.robot.climb;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberPositionCmd;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.funnel.DeployFunnelCmd;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmd;

public class StowClimberCmd extends XeroSequenceCmd {
    private ManipulatorSubsystem manipulator_ ;
    private ClimberSubsystem climber_ ;
    private FunnelSubsystem funnel_ ;

    public StowClimberCmd(ManipulatorSubsystem m, ClimberSubsystem climber, FunnelSubsystem funnel) {
        super("StowClimberCmd");
        climber_ = climber ;
        funnel_ = funnel ;
        manipulator_ = m ;
    }

    @Override
    public void initSequence(SequentialCommandGroup sequence) {
        sequence.addCommands(new ClimberPositionCmd(climber_, ClimberState.Stowed)) ;
        sequence.addCommands(new DeployFunnelCmd(funnel_, DeployFunnelCmd.Position.Normal));
        sequence.addCommands(new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;
    }
}
