package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public interface GrabberIO {
    @AutoLog
    public static class GrabberIOInputs {

        // Grabber Inputs
        public boolean grabberReady = false;
        public Angle grabberPosition = Radians.zero();
        public Current grabberCurrent = Amps.zero();
        public Voltage grabberVoltage = Volts.zero();
        public AngularVelocity grabberVelocity = RadiansPerSecond.zero();

        // Sensor Inputs
        public boolean coralSensor = false;
        public boolean coralRisingEdge = false;
        public boolean coralFallingEdge = false;

        public boolean algaeSensor = false;
        public boolean algaeRisingEdge = false;
        public boolean algaeFallingEdge = false;
    }

    public default void updateInputs(GrabberIOInputs inputs) {}

    public default void setGrabberMotorVoltage(double vol) {}

    public default void logGrabberMotor(SysIdRoutineLog log) {}

    public default void setGrabberTargetVelocity(AngularVelocity vel) {} 
    public default void setGrabberTargetPosition(Angle pos) {}

}
