package frc.robot.commands.gps;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.GamepadEnabled;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.subsystems.brain.Brain;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.CollectAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.util.ReefUtil;
import frc.robot.util.ReefUtil.ReefFace;

public class CollectReefAlgaeCmd extends SequentialCommandGroup {
    private static final Angle ArmPlaceAngle = Degrees.of(90.0) ;
    private static final Distance ElevatorPlaceHeight = Meters.of(1.0) ;

    private static Voltage nominal = Volts.of(12.0) ;

    private static final LinearVelocity PlaceMaxVelocity = MetersPerSecond.of(1.0) ;
    private static final LinearAcceleration PlaceMaxAcceleration = MetersPerSecondPerSecond.of(1.0) ;
    private static final AngularVelocity PlaceMaxAngularVelocity = DegreesPerSecond.of(60.0) ;
    private static final AngularAcceleration PlaceMaxAngularAcceleration = DegreesPerSecondPerSecond.of(60.0) ;


    private static final LinearVelocity BackupMaxVelocity = MetersPerSecond.of(1.0) ;
    private static final LinearAcceleration BackupMaxAcceleration = MetersPerSecondPerSecond.of(1.0) ;
    private static final AngularVelocity BackupMaxAngularVelocity = DegreesPerSecond.of(60.0) ;
    private static final AngularAcceleration BackupMaxAngularAcceleration = DegreesPerSecondPerSecond.of(60.0) ;

    public CollectReefAlgaeCmd(Brain b, Drive db, ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("PlaceCoralCmd") ;

        Optional<Alliance> a = DriverStation.getAlliance() ;
        if (a.isPresent()) {
            Optional<ReefFace> target = ReefUtil.getTargetedReefFace(db.getPose()) ;

            if (target.isPresent()) {
                ReefFace t = target.get() ;
                Pose2d place = t.getAlgaeScoringPose() ;
                Pose2d backup = t.getAlgaeBackupPose() ;
                
                PathConstraints place_constraints = new PathConstraints(PlaceMaxVelocity, PlaceMaxAcceleration, PlaceMaxAngularVelocity, PlaceMaxAngularAcceleration, nominal, false) ;
                PathConstraints backup_constraints = new PathConstraints(BackupMaxVelocity, BackupMaxAcceleration, BackupMaxAngularVelocity, BackupMaxAngularAcceleration, nominal, false) ;

                addCommands(
                    new GoToCmd(m, ElevatorPlaceHeight, ArmPlaceAngle),
                    new GamepadEnabled(false),
                    AutoBuilder.pathfindToPose(place, place_constraints),
                    new CollectAlgaeCmd(g),
                    AutoBuilder.pathfindToPose(backup, backup_constraints),
                    new GamepadEnabled(true),
                    new SetHoldingCmd(b, RobotContainer.GamePiece.ALGAE_HIGH),
                    new RumbleGamepadCmd(Milliseconds.of(500))) ;
            }
        }


    }
}
