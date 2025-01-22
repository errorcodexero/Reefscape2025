package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {

    }

    public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    public default void runClimberPosition(Angle angle) {}

}