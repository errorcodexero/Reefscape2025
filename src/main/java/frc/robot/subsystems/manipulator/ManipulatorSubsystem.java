package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
 
public class ManipulatorSubsystem extends SubsystemBase{
    private final ManipulatorIO io_; 
    private final ManipulatorIOInputsAutoLogged inputs_;
    private Angle target_arm_position_ ;
    private Distance target_elevator_position_ ;

    public ManipulatorSubsystem(ManipulatorIO io){
        io_ = io; 
        inputs_ = new ManipulatorIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Manipulator", inputs_);
        Logger.recordOutput("Manipulator/elevrotspos", inputs_.elevatorRawPosition.in(Rotations)) ;
        Logger.recordOutput("Manipulator/elevrotsvel", inputs_.elevatorRawVelocity.in(RotationsPerSecond)) ;
        Logger.recordOutput("Manipulator/armrotspos", inputs_.armRawPosition.in(Rotations)) ;
        Logger.recordOutput("Manipulator/armrotsvel", inputs_.armRawVelocity.in(RotationsPerSecond)) ;

        if (target_arm_position_ != null) {
            Logger.recordOutput("Manipulator/arm-target", target_arm_position_) ;
            Logger.recordOutput("Manipulator/arm-done", isArmAtTarget()) ;
        }

        if (target_elevator_position_ != null) {
            Logger.recordOutput("Manipulator/elevator-target", target_elevator_position_) ;
            Logger.recordOutput("Manipulator/elevator-done", isElevatorAtTarget()) ;
        }
    }

    public Angle getCurrentArmAngle() {
        return inputs_.armPosition ;
    }

    public void setArmAngleTarget(Angle target) {
        target_arm_position_ = target ;
        io_.setArmAngle(target);
    }

    public Distance getCurrentElevatorHeight() {
        return inputs_.elevatorPosition ;
    }

    public void setElevatorHeightTarget(Distance target) {
        target_elevator_position_ = target ;
        io_.setElevatorHeight(target);
    }

    public boolean isArmAtTarget() {
        if (target_arm_position_ == null) {
            return false ;
        }   
        return inputs_.armPosition.isNear(target_arm_position_, ManipulatorConstants.Goto.kArmTolerance) ;
    }

    public boolean isElevatorAtTarget() {
        if (target_elevator_position_ == null) {
            return false ;
        }
        return inputs_.elevatorPosition.isNear(target_elevator_position_, ManipulatorConstants.Goto.kElevatorTolerance) ;
    }

    public Command armSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return armIdRoutine().quasistatic(dir) ;
    }

    public Command armSysIdDynamic(SysIdRoutine.Direction dir) {
        return armIdRoutine().dynamic(dir) ;
    }

    public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return elevatorIdRoutine().quasistatic(dir) ;
    }
    
    public Command elevatorSysIdDynamic(SysIdRoutine.Direction dir) {
        return elevatorIdRoutine().dynamic(dir) ;
    }

    private SysIdRoutine armIdRoutine() {
        Voltage step = Volts.of(7) ;
        Time to = Seconds.of(10.0) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setArmMotorVoltage(volts.magnitude()),
                                        (log) -> io_.logArmMotor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    }

    private SysIdRoutine elevatorIdRoutine() {
        Voltage step = Volts.of(7) ;
        Time to = Seconds.of(20.0) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setElevatorMotorVoltage(volts.magnitude()),
                                        (log) -> io_.logElevatorMotor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    }
}
