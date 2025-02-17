package frc.robot.commands.robot.scorealgae;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.xerosw.util.XeroSequence;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ScoreAlgaeAfter extends Command {
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;
    private BrainSubsystem brain_ ;
    private XeroSequence sequence_ ;
    private Drive db_ ;

    public ScoreAlgaeAfter(Drive db, BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g) {
        m_ = m ;
        g_ = g ;
        brain_ = b ;
        db_ = db ;
    }

    @Override
    public void initialize() {
        sequence_ = new XeroSequence();
        sequence_.addCommands(
            new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kScoreAlgaeReef, 
                            ManipulatorConstants.Arm.Positions.kScoreAlgaeReef, true),
            new DepositAlgaeCmd(g_),
            Commands.deadline(
                new WaitCommand(1.0),
                db_.runVelocityCmd(MetersPerSecond.of(-1.0), MetersPerSecond.of(0), RadiansPerSecond.zero())),
            Commands.deadline(
                    new WaitCommand(0.1),
                    db_.runVelocityCmd(MetersPerSecond.of(0.0), MetersPerSecond.of(0), RadiansPerSecond.zero())),                
            new SetHoldingCmd(brain_, GamePiece.NONE),
            new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL3, 
                            m_.getArmPosition(), true),
            new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL3, 
                            ManipulatorConstants.Arm.Positions.kRaiseAngle, true),
            new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;
        sequence_.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            sequence_.cancel();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return sequence_.isComplete();
    }
}
