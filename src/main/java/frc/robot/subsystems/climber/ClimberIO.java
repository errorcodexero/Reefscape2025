package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {

        public Angle climberPosition;
        public AngularVelocity climberVelocity;

    }

    public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    public default void runClimberPosition(Angle angle) {}

}