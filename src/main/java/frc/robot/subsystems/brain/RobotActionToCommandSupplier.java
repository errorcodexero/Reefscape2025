package frc.robot.subsystems.brain;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.subsystems.oi.RobotAction;

public interface RobotActionToCommandSupplier {
    List<Command> get(RobotAction action, int level, CoralSide side);
}
