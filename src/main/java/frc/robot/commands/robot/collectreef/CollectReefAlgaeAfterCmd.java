package frc.robot.commands.robot.collectreef;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.xerosw.util.XeroSequence;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.commands.robot.ReportStateCmd;
import frc.robot.commands.robot.SetHoldingCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.CollectAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.util.ReefUtil;
import frc.robot.util.ReefUtil.ReefFace;

public class CollectReefAlgaeAfterCmd extends Command {

    private static Voltage nominal = Volts.of(12.0) ;

    private XeroSequence sequence_ ;
    private BrainSubsystem b_ ;
    private Drive db_ ;
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;

    public CollectReefAlgaeAfterCmd(BrainSubsystem brain, Drive db, ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("PlaceCoralCmd") ;
        db_ = db ;
        b_ = brain ;
        m_ = m ;
        g_ = g ;
    }

    @Override
    public void initialize() {
        sequence_ = new XeroSequence() ;

        Optional<Alliance> a = DriverStation.getAlliance() ;
        if (a.isEmpty())
            return ;

        Optional<ReefFace> target = ReefUtil.getTargetedReefFace(db_.getPose()) ;
        if (target.isEmpty())
            return ;

        ReefFace t = target.get() ;
        Pose2d place = t.getAlgaeScoringPose() ;
        Pose2d backup = t.getAlgaeBackupPose() ;
        
        PathConstraints driveto_constraints = new PathConstraints(CollectReefAlgaeConstants.DriveToMaxVelocity, 
                                                                    CollectReefAlgaeConstants.DriveToMaxAcceleration, 
                                                                    CollectReefAlgaeConstants.DriveToMaxAngularVelocity, 
                                                                    CollectReefAlgaeConstants.DriveToMaxAngularAcceleration, 
                                                                    nominal, false) ;

        PathConstraints backup_constraints = new PathConstraints(CollectReefAlgaeConstants.DriveToMaxVelocity, 
                                                                    CollectReefAlgaeConstants.DriveToMaxAcceleration, 
                                                                    CollectReefAlgaeConstants.DriveToMaxAngularVelocity, 
                                                                    CollectReefAlgaeConstants.DriveToMaxAngularAcceleration, 
                                                                    nominal, false) ;                                                                

        ParallelCommandGroup parallel = new ParallelCommandGroup() ;
        parallel.addCommands(
            new CollectAlgaeCmd(g_),
            AutoBuilder.pathfindToPose(place, driveto_constraints)) ;

        sequence_.addCommands(
            new ReportStateCmd(getName(), "goto"),
            new GoToCmd(m_, CollectReefAlgaeConstants.Collect.ElevatorHeight[b_.level()], CollectReefAlgaeConstants.Collect.ArmAngle[b_.level()]),
            new ReportStateCmd(getName(), "parallel"),
            parallel,
            new SetHoldingCmd(b_, RobotContainer.GamePiece.ALGAE_HIGH),
            new ReportStateCmd(getName(), "drive-backup"),
            AutoBuilder.pathfindToPose(backup, backup_constraints),
            new ReportStateCmd(getName(), "rumble"),
            new RumbleGamepadCmd(Milliseconds.of(500))) ;

        sequence_.schedule();
    }

    @Override
    public boolean isFinished() {
        return sequence_.isComplete() ;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            sequence_.cancel() ;
        }
    }
}
