package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.*;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {
        //climber inputs
        public Angle climberPosition = Degrees.of(0);
        public AngularVelocity climberVelocity = RadiansPerSecond.of(0);
        public Voltage climberVoltage = Volts.of(0);
        public Current climberCurrent = Amps.of(0);

        public boolean attachedSensor = false ;
        
        public double absEncoderRawValue = 0.0 ;
        public Angle absEncoderValue = Rotations.zero() ;
    }

    //Update Inputs
    public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    public default void setClimberPosition(Angle angle) {}

}