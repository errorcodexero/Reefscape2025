package frc.robot.subsystems.grabber;

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
        public Angle grabberPosition;
        public Current grabberCurrent;
        public Voltage grabberVoltage;
        public AngularVelocity grabberVelocity;

        // Sensor Inputs
        public boolean coralSensor;
        public boolean coralRisingEdge;
        public boolean coralFallingEdge;

        public boolean algaeSensor;
        public boolean algaeRisingEdge;
        public boolean algaeFallingEdge;
    }

    public default void updateInputs(GrabberIOInputs inputs) {}

    public default void setGrabberMotorVoltage(double vol) {}

    public default void logGrabberMotor(SysIdRoutineLog log) {}

    public default void setGrabberTargetVelocity(double vel) {} 

}
