package frc.robot.subsystems.algaeManipulator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public interface AlgaeManipulatorIO{
    @AutoLog
    public static class AlgaeManipulatorIOInputs{
        public Angle hingeAngle;
        public AngularVelocity hingeVelocity;
        public Current hingeCurrent;
        public Voltage hingeVoltage; 

        public Angle rollerAngle;
        public AngularVelocity rollerVelocity;
        public Current rollerCurrent;
        public Voltage rollerVoltage;
    }

    public default void updateInputs(AlgaeManipulatorIOInputsAutoLogged inputs_){
        
    }

    public default void setHingeMotorVoltage(double vol) {
    }

    public default double getHingeMotorVoltage(){
        return 0.0;
    }

    public default void logHingeMotor(SysIdRoutineLog log) {
    }

    public default void setRollerMotorVoltage(double vol) {
    }

    public default double getRollerMotorVoltage(){
        return 0.0;
    }

    public default void logRollerMotor(SysIdRoutineLog log) {
    }

    public default void simulate() {
    }
}
