package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*; 

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
 
public class ManipulatorSubsystem extends SubsystemBase{
    private final ManipulatorIO io_; 
    private final ManipulatorIOInputsAutoLogged inputs_;  
    private Angle target_angle_;
    private Distance target_height_;
    private boolean elevator_reset_ ;
    private Trigger needs_reset_trigger_ ;

    private final Alert armDisconnected_ = new Alert("Arm motor failed to configure or is disconnected!", AlertType.kError);
    private final Alert elevator1Disconnected_ = new Alert("Elevator motor 1 failed to configure or is disconnected!", AlertType.kError);
    private final Alert elevator2Disconnected_ = new Alert("Elevator motor 2 failed to configure or is disconnected!", AlertType.kError);

    public ManipulatorSubsystem(ManipulatorIO io) {
        io_ = io; 
        inputs_ = new ManipulatorIOInputsAutoLogged(); 
        elevator_reset_ = false ;

        needs_reset_trigger_ = new Trigger(() -> !elevator_reset_) ;
    }

    public Trigger needsElevatorReset() { 
        return new Trigger(()-> !elevator_reset_) ;
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
    }

    public Angle getArmPosition() {
        return inputs_.armPosition;
    }

    public Angle getArmTarget() {
        return target_angle_;
    }

    public void setArmTarget(Angle angle) {
        target_angle_ = angle;  
        io_.setArmTarget(angle); 
    }

    public LinearVelocity getElevatorVelocity() {
        return inputs_.elevatorVelocity ;
    }

    public void resetElevator() {
        elevator_reset_ = true ;
        io_.resetPosition() ;
    }

    public Distance getElevatorPosition() {
        return inputs_.elevatorPosition; 
    }

    public Distance getElevatorTarget() {
        return target_height_;
    }

    public void setElevatorTarget(Distance dist) {
        target_height_ = dist;
        io_.setElevatorTarget(dist); 
    }

    public void resetPosition() {
        io_.resetPosition();
    }

    public void setElevatorVoltage(Voltage volts) {
        io_.setElevatorMotorVoltage(volts.in(Volts)) ;
    }

    public boolean isElevAtBottom() {
        return inputs_.hallEffectSensor ;
    }

    public boolean isElevAtTarget() {
        if (target_height_ == null)
            return false;

        if (!inputs_.elevatorPosition.isNear(target_height_, ManipulatorConstants.Elevator.kPosTolerance))
            return false ;

        if (Robot.isReal() && !inputs_.elevatorVelocity.isNear(MetersPerSecond.of(0.0), ManipulatorConstants.Elevator.kVelTolerance))
            return false ;

        return true ;
    }

    public boolean isArmAtTarget() {
        if (target_angle_ == null)
            return false;            

        if (!inputs_.armPosition.isNear(target_angle_, ManipulatorConstants.Arm.kPosTolerance))
            return false ;

        if (Robot.isReal() && !inputs_.armVelocity.isNear(RotationsPerSecond.of(0), ManipulatorConstants.Arm.kVelTolerance))
            return false ;

        return true ;
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