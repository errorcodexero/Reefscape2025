package frc.robot.commands.robot.climb;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.ClimbCmd;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;

public class ExecuteClimbCmd extends XeroSequenceCmd {
    private ClimberSubsystem climber_;
    private Drive drive_ ;
    private LinearVelocity vel_ ;
    private Time length_ ;

    public ExecuteClimbCmd(ClimberSubsystem climber, Drive drive, LinearVelocity vel, Time length) {
        super("ExecuteClimbCmd");

        climber_ = climber;
        drive_ = drive ;
        vel_ = vel ;
        length_ = length ;
    }

    @Override
    public void initSequence(SequentialCommandGroup sequence) {
        if (length_.gt(Seconds.zero()) && vel_.gt(MetersPerSecond.zero())) {
            sequence.addCommands(
                Commands.deadline(
                    new WaitCommand(length_),
                    drive_.runVelocityCmd(new ChassisSpeeds(0.0, vel_.in(MetersPerSecond), 0.0)))
            ) ;
        }

        sequence.addCommands(new ClimbCmd(climber_));
    }
}
