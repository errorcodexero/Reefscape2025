package frc.robot.commands.gps;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.GamepadEnabled;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.subsystems.brain.Brain;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.util.ReefUtil;
import frc.robot.util.ReefUtil.ReefFace;

public class PlaceCoralAfterCmd extends Command {

    private static Voltage nominal = Volts.of(12.0) ;

    private SequentialCommandGroup sequence_ ;
    private Brain b_ ;
    private Drive db_ ;
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;
    private boolean automode_ ;

    public PlaceCoralAfterCmd(Brain brain, Drive db, ManipulatorSubsystem m, GrabberSubsystem g, boolean automode) {
        setName("PlaceCoralCmd") ;
        db_ = db ;
        b_ = brain ;
        m_ = m ;
        g_ = g ;
        automode_ = automode ;
        sequence_ = new SequentialCommandGroup() ;
    }

    @Override
    public void initialize() {
        Optional<Alliance> a = DriverStation.getAlliance() ;
        if (a.isPresent()) {
            Optional<ReefFace> target = ReefUtil.getTargetedReefFace(db_.getPose()) ;

            if (target.isPresent()) {
                ReefFace t = target.get() ;
                Pose2d place = (b_.coralSide() == CoralSide.Left) ? t.getLeftScoringPose() : t.getRightScoringPose() ;
                Pose2d backup = (b_.coralSide() == CoralSide.Left) ? t.getLeftBackupPose() : t.getRightBackupPose() ; 
                
                PathConstraints driveto_constraints = new PathConstraints(CommandPositions.DriveToMaxVelocity, 
                                                                        CommandPositions.DriveToMaxAcceleration, 
                                                                        CommandPositions.DriveToMaxAngularVelocity, 
                                                                        CommandPositions.DriveToMaxAngularAcceleration, nominal, false) ;
                PathConstraints backup_constraints = new PathConstraints(CommandPositions.BackupMaxVelocity, 
                                                                         CommandPositions.BackupMaxAcceleration, 
                                                                         CommandPositions.BackupMaxAngularVelocity, 
                                                                         CommandPositions.BackupMaxAngularAcceleration, nominal, false) ;

                if (!automode_) {
                    sequence_.addCommands(
                        new ReportStateCmd(getName(), "goto"),
                        new GoToCmd(m_, CommandPositions.Place.ElevatorHeight[b_.coralLevel()], CommandPositions.Place.ArmAngle[b_.coralLevel()]),
                        new ReportStateCmd(getName(), "gp-disabled"),
                        new GamepadEnabled(false),
                        new ReportStateCmd(getName(), "driveto"),
                        AutoBuilder.pathfindToPose(place, driveto_constraints)) ;
                }

                sequence_.addCommands(
                    new ReportStateCmd(getName(), "deposit-coral"),
                    new DepositCoralCmd(g_),
                    new SetHoldingCmd(b_, RobotContainer.GamePiece.NONE)) ;

                if (!automode_) {
                    sequence_.addCommands(
                        new ReportStateCmd(getName(), "backup"),
                        AutoBuilder.pathfindToPose(backup, backup_constraints),
                        new ReportStateCmd(getName(), "gamepad-enabled"),
                        new GamepadEnabled(true),
                        new ReportStateCmd(getName(), "set-holding"),
                        new SetHoldingCmd(b_, RobotContainer.GamePiece.NONE),
                        new ReportStateCmd(getName(), "rumble"),
                        new RumbleGamepadCmd(Milliseconds.of(500))) ;
                }
            }
        }

        sequence_.schedule();
    }

    @Override
    public boolean isFinished() {
        return sequence_.isFinished() ;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            sequence_.cancel() ;
        }
    }
}
