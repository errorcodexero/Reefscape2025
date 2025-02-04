package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;

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
        public Angle armPosition = Degrees.of(0); 
        public Current armCurrent = Amps.of(0); 
        public Voltage armVoltage = Volts.of(0); 
        public AngularVelocity armVelocity = RadiansPerSecond.of(0); 

        // elevator inputs
        public Distance elevatorPosition = Meters.of(0); 
        public Current elevatorCurrent = Amps.of(0);  
        public Voltage elevatorVoltage = Volts.of(0);
        public LinearVelocity elevatorVelocity = MetersPerSecond.of(0); 

        public Voltage elevator2Voltage = Volts.of(0);
        public Current elevator2Current = Amps.of(0); 
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
