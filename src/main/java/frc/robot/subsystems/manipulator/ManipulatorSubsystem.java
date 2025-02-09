package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*; 

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
 
public class ManipulatorSubsystem extends SubsystemBase{
    private final ManipulatorIO io_; 
    private final ManipulatorIOInputsAutoLogged inputs_;  
    private Angle target_angle_;
    private Distance target_height_;

    private final Alert armDisconnected_ = new Alert("Arm motor failed to configure or is disconnected!", AlertType.kError);
    private final Alert elevator1Disconnected_ = new Alert("Elevator motor 1 failed to configure or is disconnected!", AlertType.kError);
    private final Alert elevator2Disconnected_ = new Alert("Elevator motor 2 failed to configure or is disconnected!", AlertType.kError);

    public ManipulatorSubsystem(ManipulatorIO io) {
        io_ = io; 
        inputs_ = new ManipulatorIOInputsAutoLogged(); 
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Manipulator", inputs_);
        Logger.recordOutput("Manipulator/ArmTarget", target_angle_) ;
        Logger.recordOutput("Manipulator/ElevatorTarget", target_height_) ;
        Logger.recordOutput("Manipulator/armReady", isArmAtTarget()) ;
        Logger.recordOutput("Manipulator/elevatorReady", isElevAtTarget()) ;

        armDisconnected_.set(!inputs_.armReady);
        elevator1Disconnected_.set(!inputs_.elevator1Ready);
        elevator2Disconnected_.set(!inputs_.elevator2Ready);

        Logger.recordOutput("Manipulator/elevRawPos", inputs_.elevatorRawMotorPosition.in(Rotations)) ;
        Logger.recordOutput("Manipulator/elevRawVel", inputs_.elevatorRawMotorVelocity.in(RotationsPerSecond)) ;
        Logger.recordOutput("Manipulator/armRawPos", inputs_.armRawMotorPosition.in(Rotations)) ;
        Logger.recordOutput("Manipulator/armRawVel", inputs_.armRawMotorVelocity.in(RotationsPerSecond)) ;        
    }

    public Angle getArmPosition() {
        return inputs_.armPosition;
    }

    public void setArmPosition(Angle angle) {
        target_angle_ = angle;  
        io_.setArmTarget(angle); 
    }

    public Distance getElevatorPosition() {
        return inputs_.elevatorPosition; 
    }

    public void setElevatorPosition(Distance dist) {
        target_height_ = dist;
        io_.setElevatorPosition(dist); 
    }

    public boolean doesCrossKZ(Angle current, Angle target) {
        return (current.lt(ManipulatorConstants.Keepout.kKeepoutMinAngle) && target.gt(ManipulatorConstants.Keepout.kKeepoutMaxAngle)) || 
               (current.gt(ManipulatorConstants.Keepout.kKeepoutMaxAngle) && target.lt(ManipulatorConstants.Keepout.kKeepoutMinAngle)) ;
    }

    public boolean isElevAtTarget() {
        if (target_height_ == null)
            return false;

        return inputs_.elevatorPosition.isNear(target_height_, ManipulatorConstants.Elevator.kPosTolerance) && 
               inputs_.elevatorVelocity.isNear(MetersPerSecond.of(0.0), ManipulatorConstants.Elevator.kVelTolerance);
    }

    public boolean isArmAtTarget() {
        if (target_angle_ == null)
            return false;            

        return inputs_.armPosition.isNear(target_angle_, ManipulatorConstants.Arm.kPosTolerance) && 
               inputs_.armVelocity.isNear(RotationsPerSecond.of(0), ManipulatorConstants.Arm.kVelTolerance);
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