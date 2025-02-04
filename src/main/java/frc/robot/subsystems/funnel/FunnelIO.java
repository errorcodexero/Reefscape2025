package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface FunnelIO {
    @AutoLog
    public class FunnelIOInputs {
        // funnel drop motor inputs
        public Angle funnelPosition ;
        public AngularVelocity funnelVelocity ;
        public Voltage funnelVoltage ;
        public Current funnelCurrent ;

    }
    //Update Inputs
    public default void updateInputs(FunnelIOInputsAutoLogged inputs) {}

    //Set Target Position
    public default void setTargetPosition(Angle pos) {}
}