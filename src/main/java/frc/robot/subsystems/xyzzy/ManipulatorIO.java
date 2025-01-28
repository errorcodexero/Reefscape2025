package frc.robot.subsystems.Manipulator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
 
public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs{
        // inputs for now: position, current, voltage, angular velocity (and add acceleration?)

        // arm inputs
        public Angle armPosition; 
        public Current armCurrent; 
        public Voltage armVoltage; 
        public AngularVelocity armVelocity; 

        // elevator inputs
        public Angle elevatorPosition; 
        public Current elevatorCurrent; 
        public Voltage elevatorVoltage; 
        public AngularVelocity elevatorVelocity; 
    }

    // updating inputs
    public default void updateInputs(ManipulatorIOInputs inputs) {}

    // Needed for SYS Id support
    public default void setArmMotorVoltage(double vol) {}

    public default void logArmMotor(SysIdRoutineLog log) {}

}
