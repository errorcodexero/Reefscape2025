package frc.robot.commands.gps;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.WaitForCoralCmd;
import frc.robot.subsystems.manipulator.ManipulatorGotoCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.RobotContainer;

public class CollectCoralCmd extends SequentialCommandGroup {

    private static final Angle ArmCollectAngle = Degrees.of(90.0) ;
    private static final Distance ElevatorCollectHeight = Meters.of(1.0) ;

    public CollectCoralCmd(ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("CollectCoralCmd") ;

        addCommands(
            new ReportStateCmd(getName(), "goto"),
            new ParallelDeadlineGroup(
                new WaitForCoralCmd(g),
                new ManipulatorGotoCmd(m, ElevatorCollectHeight, ArmCollectAngle)),
            new ReportStateCmd(getName(), "holding"),
            new SetHoldingCmd(RobotContainer.GamePiece.CORAL),
            new ReportStateCmd(getName(), "rumbling"),
            new RumbleGamepadCmd(Milliseconds.of(500)),
            new ReportStateCmd(getName(), "done")) ;
    }
}
