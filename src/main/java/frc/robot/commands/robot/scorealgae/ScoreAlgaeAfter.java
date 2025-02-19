package frc.robot.commands.robot.scorealgae;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.CommandConstants;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ScoreAlgaeAfter extends XeroSequenceCmd {
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;
    private BrainSubsystem brain_ ;
    private Drive db_ ;

    public final static Pose2d kProcessorRedPose = new Pose2d() ;
    public final static Pose2d kProcessorBluePose = new Pose2d() ;

    public ScoreAlgaeAfter(Drive db, BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g) {
        m_ = m ;
        g_ = g ;
        brain_ = b ;
        db_ = db ;
    }

    @Override
    public void initSequence(SequentialCommandGroup seq) {
        Optional<Alliance> alliance = DriverStation.getAlliance() ;
        if (alliance.isPresent()) {
            Pose2d scoringPose = alliance.get() == Alliance.Red ? kProcessorRedPose : kProcessorBluePose ;

            seq.addCommands(
                RobotContainer.getInstance().gamepad().setLockCommand(true),
                Commands.parallel(
                    new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kScoreAlgaeReef, 
                                    ManipulatorConstants.Arm.Positions.kScoreAlgaeReef, true),
                    DriveCommands.simplePathCommand(db_, scoringPose, CommandConstants.AlgaeScore.kMaxDriveVelocity,
                                                        CommandConstants.AlgaeScore.kMaxDriveAcceleration)),
                new DepositAlgaeCmd(g_),
                Commands.deadline(
                    new WaitCommand(1.0),
                    db_.runVelocityCmd(MetersPerSecond.of(-1.0), MetersPerSecond.of(0), RadiansPerSecond.zero())),
                Commands.deadline(
                        new WaitCommand(0.02),
                        db_.runVelocityCmd(MetersPerSecond.of(0.0), MetersPerSecond.of(0), RadiansPerSecond.zero())),                
                new SetHoldingCmd(brain_, GamePiece.NONE),
                new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL3, 
                                m_.getArmPosition(), true),
                new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL3, 
                                ManipulatorConstants.Arm.Positions.kRaiseAngle, true),
                new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;
        }
    }
}
