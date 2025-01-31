package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {

        //climber inputs
        public Angle climberPosition;
        public AngularVelocity climberVelocity;
        public Voltage climberVoltage;
        public Current climberCurrent;

    }
    //Update Inputs
    public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

}