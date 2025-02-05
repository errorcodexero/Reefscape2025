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
        public Angle grabberPositionCoralSensorLowEdge ;

        public boolean coralHigh ;
        public boolean coralHighRisingEdge ;
        public boolean coralHighFallingEdge ;
        public Angle grabberPositionCoralSensorHighEdge ;

        public boolean coralFunnel ;
        public boolean coralFunnelRisingEdge ;
        public boolean coralFunnelFallingEdge ;

        public boolean algaeHigh ;
        public boolean algaeHighRisingEdge ;
        public boolean algaeHighFallingEdge;

        public boolean algaeLow ;
        public boolean algaeLowRisingEdge ;
        public boolean algaeLowFallingEdge ;
    }

    // update all inputs
    public default void updateInputs(GrabberIOInputs inputs) {}

    // grabber related methods
    public default void setGrabberTargetVelocity(AngularVelocity target) {}
    public default void setGrabberTargetPosition(Angle target) {}
    public default void setGrabberMotorVoltage(double volts) {}
    public default void logGrabberMotor(SysIdRoutineLog log) {} 
}
