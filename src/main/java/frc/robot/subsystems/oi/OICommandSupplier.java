package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj2.command.Command;

public interface OICommandSupplier {
    public record Pair<One, Two>(One one, Two two) {
    }

    Pair<Command, Command> get(RobotAction action, int level, CoralSide side);
}
