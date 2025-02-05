package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public interface GrabberIO {
    @AutoLog
    public static class GrabberIOInputs{
        // grabber inputs
        public Current grabberCurrent ;
        public Voltage grabberVoltage ;
        public Angle grabberPosition ;
        public AngularVelocity grabberVelocity ;
        public AngularVelocity grabberRawVelocity ;

        public boolean coralSensorLow ;
        public boolean coralSensorLowRisingEdge ;
        public boolean coralSensorLowFallingEdge ;
        public double coralSensorPositionLow ;

        public boolean coralSensorHigh ;
        public boolean coralSensorHighRisingEdge ;
        public boolean coralSensorHighFallingEdge ;
        public double coralSensorPositionHigh ;

        public boolean coralSensorFunnel ;
        public boolean coralSensorFunnelRisingEdge ;
        public boolean coralSensorFunnelFallingEdge ;

        public boolean algaeSensorHigh ;
        public boolean algaeSensorRisingEdgeHigh ;
        public boolean algaeSensorFallingEdgeHigh;

        public boolean algaeSensorLow ;
        public boolean algaeSensorRisingEdgeLow ;
        public boolean algaeSensorFallingEdgeLow ;
    }

    // update all inputs
    public default void updateInputs(GrabberIOInputs inputs) {}

    // grabber related methods
    public default void setGrabberVelocity(AngularVelocity target) {}
    public default void setGrabberMotorVoltage(double volts) {}
    public default void logGrabberMotor(SysIdRoutineLog log) {} ;    
}
