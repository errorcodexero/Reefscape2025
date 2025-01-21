package frc.robot.subsystems.CoralPlacer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
 
public interface CoralPlacerIO {
    @AutoLog
    public static class CoralPlacerIOInputs{
        // inputs for now: position, current, voltage, acceleration, angular velocity
        //
        // assuming the coral placer is one component for now, will split up into 
        // arm, grabber, etc. as design is finalized

        public Angle coralPlacerPosition; 
        public Current coralPlacerCurrent; 
        public Voltage coralPlacerVoltage; 
        public Acceleration coralPlacerAcceleration; 
        public AngularVelocity coralPlacerVelocity; 
    }

    // updating inputs
    public default void updateInputs(CoralPlacerIOInputs inputs) {}

    // Needed for SYS Id support
    public default void setArmMotorVoltage(double vol) {}

    public default void logArmMotor(SysIdRoutineLog log) {}

    // For the simulator
    public default void simulate() {}
}
