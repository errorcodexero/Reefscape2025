package frc.robot.commands.robot.algaenet;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;

import org.xerosw.util.XeroSequenceCmd;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.util.ReefUtil;

public class AlgaeNetDriveCmd extends XeroSequenceCmd {
    static LinearVelocity kMaxVel = MetersPerSecond.of(2.0) ;
    static LinearAcceleration kMaxAcc = MetersPerSecondPerSecond.of(2.0) ;

    private BrainSubsystem b_ ;
    private Drive db_ ;
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;

    public AlgaeNetDriveCmd(BrainSubsystem b, Drive db, ManipulatorSubsystem m, GrabberSubsystem g) {
        super("AlgaeNetDriveCmd") ;
        b_ = b ;
        db_ = db ;
        m_ = m ;
        g_ = g ;
    }

    @Override
    public void initSequence(SequentialCommandGroup seq) {
        Pose2d target = ReefUtil.getBargeScorePose(db_.getPose()) ;
        if (target.getTranslation().getDistance(db_.getPose().getTranslation()) < 2.0) {
            seq.addCommands(
                DriveCommands.simplePathCommand(db_, target, kMaxVel, kMaxAcc),
                new WaitCommand(Milliseconds.of(500)),
                new AlgaeNetCmd(b_, m_, g_)
            ) ;        
        }
    }    
}
