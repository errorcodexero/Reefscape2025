package frc.robot.subsystems.funnel;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface FunnelIO {
    
    @AutoLog
    public static class FunnelInputs {

        public boolean funnelReady = false;
        public Voltage funnelVoltage = Volts.zero();
        public Current funnelCurrent = Amps.zero();
        public Angle funnelRawPosition = Rotations.zero();
        public AngularVelocity funnelRawVelocity = RotationsPerSecond.zero();
        public Angle funnelPosition = Rotations.zero();
        public AngularVelocity funnelVelocity = RotationsPerSecond.zero();

        public boolean coralFunnelSensor = false;
        public boolean coralFunnelRisingEdge = false;
        public boolean coralFunnelFallingEdge = false;

        public double absEncoderRawValue = 0 ;
        public Angle absEncoderValue = Rotations.zero() ;

    }

    public default void updateInputs(FunnelInputs inputs) {};

    public default void setPosition(Angle angle) {};

}
