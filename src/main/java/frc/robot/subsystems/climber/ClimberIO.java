package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {

        // climber arm inputs
        public Angle climberPosition;
        public AngularVelocity climberVelocity;
        public Voltage climberVoltage;
        public Current climberCurrent;

        public boolean cage_sensor_1_ ;
        public boolean cage_sensor_2_ ;
        public boolean cage_sensor_3_ ;

        public boolean door_sensor_1_ ;
        public boolean door_sensor_2_ ;

    }
    //Update Inputs
    public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}
    public default void setClimberPosition(Angle target) { }
}