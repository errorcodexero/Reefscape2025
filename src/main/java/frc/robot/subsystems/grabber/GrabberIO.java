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

        public boolean grabberSensor ;
        public boolean risingEdge ;
        public boolean fallingEdge ;
        public Angle posOnEdge ;
    }

    // update all inputs
    public default void updateInputs(GrabberIOInputs inputs) {}

    // grabber related methods
    public default void setGrabberVelocity(AngularVelocity target) {}
    public default void setGrabberMotorVoltage(double volts) {}
    public default void logGrabberMotor(SysIdRoutineLog log) {} ;    
}
