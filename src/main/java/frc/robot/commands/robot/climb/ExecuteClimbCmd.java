package frc.robot.commands.robot.climb;

import static edu.wpi.first.units.Units.Seconds;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.ClimberPositionCmd;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;

public class ExecuteClimbCmd extends XeroSequenceCmd {
    private ClimberSubsystem climber_;
    private Drive drive_ ;

    public ExecuteClimbCmd(ClimberSubsystem climber, Drive drive) {
        super("ExecuteClimbCmd");

        climber_ = climber;
        drive_ = drive ;
    }

    @Override
    public void initSequence(SequentialCommandGroup sequence) {
        sequence.addCommands(
            Commands.deadline(
                new WaitCommand(Seconds.of(1.25)),
                drive_.runVelocityCmd(new ChassisSpeeds(0.0, 0.25, 0.0)))
        ) ;
        sequence.addCommands(new ClimberPositionCmd(climber_, ClimberState.Climb));
    }
}
