package frc.robot.commands.gps;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.GamepadEnabled;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.DepositCoralCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorGotoCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.util.ReefscapeMath;
import frc.robot.util.ReefscapeMath.ReefPlaceInfo;

public class PlaceCoralCmd extends SequentialCommandGroup {
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

    public PlaceCoralCmd(Drive db, ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("PlaceCoralCmd") ;

        ReefPlaceInfo info = ReefscapeMath.findReefInfo() ;

        PathConstraints place_constraints = new PathConstraints(PlaceMaxVelocity, PlaceMaxAcceleration, PlaceMaxAngularVelocity, PlaceMaxAngularAcceleration, nominal, false) ;
        PathConstraints backup_constraints = new PathConstraints(BackupMaxVelocity, BackupMaxAcceleration, BackupMaxAngularVelocity, BackupMaxAngularAcceleration, nominal, false) ;

        addCommands(
            new ManipulatorGotoCmd(m, ElevatorPlaceHeight, ArmPlaceAngle),
            AutoBuilder.pathfindToPose(info.place_pose, place_constraints),
            new GamepadEnabled(false),
            new DepositCoralCmd(g),
            AutoBuilder.pathfindToPose(info.backup_pose, backup_constraints),
            new GamepadEnabled(true),
            new RumbleGamepadCmd(Milliseconds.of(500))) ;
    }
}
