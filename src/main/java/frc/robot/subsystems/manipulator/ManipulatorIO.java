package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
 
public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs {
        // inputs for now: position, current, voltage, angular velocity (and add acceleration?)

        // arm inputs
        public Angle armPosition; 
        public Current armCurrent; 
        public Voltage armVoltage; 
        public AngularVelocity armVelocity; 

        // elevator inputs
        public Distance elevatorPosition; 
        public Current elevatorCurrent; 
        public Voltage elevatorVoltage; 
        public AngularVelocity elevatorVelocity; 
    }

    // updating inputs
    public void updateInputs(ManipulatorIOInputs inputs); 

    // Needed for SYS Id support
    public default void setArmMotorVoltage(double vol){}

    public default void logArmMotor(SysIdRoutineLog log){}

    // ELEVATOR METHODS
    public void setElevatorPosition(double m); 

    public double getElevatorFrontPosition();

    public double getElevatorBackPosition();

    // ARM METHODS
    public void setArmPosition(double deg); 

    public double getArmPosition();

}
