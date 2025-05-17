package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
 
public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs {
        // inputs for now: position, current, voltage, velocity (acceleration?)

        // arm inputs
        public boolean armReady = false;
        public Angle armPosition = Degrees.of(0); 
        public Current armCurrent = Amps.of(0); 
        public Voltage armVoltage = Volts.of(0); 
        public AngularVelocity armVelocity = RadiansPerSecond.of(0); 
        public Angle armRawMotorPosition = Degrees.of(0);
        public AngularVelocity armRawMotorVelocity = DegreesPerSecond.of(0.0) ;
        public int syncCount = Integer.MAX_VALUE ;
      
        // elevator
        public Distance elevatorPosition = Meters.of(0); 
        public LinearVelocity elevatorVelocity = MetersPerSecond.of(0); 
        public Angle elevatorRawMotorPosition = Degrees.of(0);
        public AngularVelocity elevatorRawMotorVelocity = DegreesPerSecond.of(0.0) ;

        // elevator 1
        public boolean elevator1Ready = false;
        public Voltage elevator1Voltage = Volts.of(0);
        public Current elevator1Current = Amps.of(0);  
        public Power elevator1Power = Watts.zero() ;
        public Power elevator1PowerAvg = Watts.zero() ;
        

        // elevator 2
        public boolean elevator2Ready = false;
        public Voltage elevator2Voltage = Volts.of(0);
        public Current elevator2Current = Amps.of(0); 
        public Power elevator2Power = Watts.zero() ;
        public Power elevator2PowerAvg = Watts.zero() ;

        // encoder
        public Angle absoluteEncoder = Degrees.of(0); 
        public double rawAbsoluteEncoder = 0;
    }

    // updating inputs
    public default void updateInputs(ManipulatorIOInputs inputs) {}

    public default void toggleSyncing() {}

    // Needed for SYS ID support
    public default void setArmMotorVoltage(double vol) {}

    public default void logArmMotor(SysIdRoutineLog log) {}

    public default void setElevatorMotorVoltage(double vol) {}

    public default void logElevatorMotor(SysIdRoutineLog log) {}

    // ELEVATOR METHODS
    public default void setElevatorTarget(Distance dist) {}
    public default void setElevatorPosition(Distance d) {} ;

    // ARM METHODS
    public default void setArmTarget(Angle angle) {}

}
