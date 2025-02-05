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
    public static class ManipulatorIOInputs{
        // inputs for now: position, current, voltage, angular velocity (and add acceleration?)

        // arm inputs
        public Angle armPosition; 
        public Angle armRawPosition ;
        public AngularVelocity armRawVelocity ;
        public double armRawEncoder ;
        public Angle armEncoderValue ;
        public Current armCurrent; 
        public Voltage armVoltage; 
        public AngularVelocity armVelocity; 

        // elevator inputs
        public Distance elevatorPosition;
        public Angle elevatorRawPosition ;
        public AngularVelocity elevatorRawVelocity ;
        public Current elevatorCurrent; 
        public Voltage elevatorVoltage; 
        public LinearVelocity elevatorVelocity;
    }

    // update all inputs
    public default void updateInputs(ManipulatorIOInputs inputs) {}

    // arm related methods
    public default void setArmAngle(Angle target) {}
    public default void setArmMotorVoltage(double volts) {}
    public default void logArmMotor(SysIdRoutineLog log) {}

    // elevator related methods
    public default void setElevatorHeight(Distance target) {}
    public default void setElevatorMotorVoltage(double volts) {}
    public default void logElevatorMotor(SysIdRoutineLog log) {} ;
}
