package frc.robot.commands.robot.algaenet;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;

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

public class AlgaeNetDriveCmd extends SequentialCommandGroup {
    static LinearVelocity kMaxVel = MetersPerSecond.of(2.0) ;
    static LinearAcceleration kMaxAcc = MetersPerSecondPerSecond.of(2.0) ;

    public AlgaeNetDriveCmd(BrainSubsystem b, Drive db, ManipulatorSubsystem m, GrabberSubsystem g) {
        Pose2d target = ReefUtil.getBargeScorePose(db.getPose()) ;
        if (target.getTranslation().getDistance(db.getPose().getTranslation()) < 2.0) {
            addCommands(
                DriveCommands.simplePathCommand(db, target, kMaxVel, kMaxAcc),
                new WaitCommand(Milliseconds.of(500)),
                new AlgaeNetCmd(b, m, g)
            ) ;        
        }
    }    
}
