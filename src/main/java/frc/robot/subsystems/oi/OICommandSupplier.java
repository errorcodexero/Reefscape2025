package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.oi.OISubsystem.CoralSide;
import frc.robot.subsystems.oi.OISubsystem.RobotAction;

public interface OICommandSupplier {
    Command get(RobotAction action, int level, CoralSide side);
}
