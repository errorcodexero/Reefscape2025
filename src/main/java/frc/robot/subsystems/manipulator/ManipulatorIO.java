package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
 
public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs {
        // inputs for now: position, current, voltage, velocity (acceleration?)

        // arm inputs
        public boolean armReady = false;
        public Angle armPosition; 
        public Current armCurrent; 
        public Voltage armVoltage; 
        public AngularVelocity armVelocity; 

        // elevator inputs
        public boolean elevatorReady = false;
        public Distance elevatorPosition; 
        public Current elevatorCurrent; 
        public Voltage elevatorVoltage; 
        public LinearVelocity elevatorVelocity; 
    }

    // updating inputs
    public default void updateInputs(ManipulatorIOInputs inputs) {}

    // Needed for SYS ID support
    public default void setArmMotorVoltage(double vol) {}

    public default void logArmMotor(SysIdRoutineLog log) {}

    public default void setElevatorMotorVoltage(double vol) {}

    public default void logElevatorMotor(SysIdRoutineLog log) {}

    // ELEVATOR METHODS
    public default void setElevatorPosition(Distance dist) {}

    // ARM METHODS
    public default void setArmPosition(Angle angle) {}

}
