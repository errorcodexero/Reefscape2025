package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*; 

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
 
public class ManipulatorSubsystem extends SubsystemBase {
    private final ManipulatorIO io_; 
    private final ManipulatorIOInputsAutoLogged inputs_;  
    private Angle arm_target_;
    private Angle arm_pos_tolerance_ ;
    private AngularVelocity arm_vel_tolerance_ ;

    private Distance elev_target_;
    private Distance elev_pos_tolerance_ ;
    private LinearVelocity elev_vel_tolerance_ ;

    private boolean elevator_calibrated_ ;
    
    private final Alert armDisconnected_ = new Alert("Arm motor failed to configure or is disconnected!", AlertType.kError);
    private final Alert elevator1Disconnected_ = new Alert("Elevator motor 1 failed to configure or is disconnected!", AlertType.kError);
    private final Alert elevator2Disconnected_ = new Alert("Elevator motor 2 failed to configure or is disconnected!", AlertType.kError);

    public ManipulatorSubsystem(ManipulatorIO io) {
        io_ = io; 
        inputs_ = new ManipulatorIOInputsAutoLogged(); 
        elevator_calibrated_ = false ;
    }

    public boolean isElevatorCalibrated() {
        return elevator_calibrated_ ;
    }

    public void toggleSyncing() {
        io_.toggleSyncing() ;
    }

    @Override
    public void periodic() {

        io_.updateInputs(inputs_);
        Logger.processInputs("Manipulator", inputs_);

        Logger.recordOutput("Manipulator/calibrated", elevator_calibrated_) ;

        Logger.recordOutput("Manipulator/ArmTarget", arm_target_) ;
        Logger.recordOutput("Manipulator/ElevatorTarget", elev_target_) ;

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
        return arm_target_;
    }

    public void setArmTarget(Angle angle) {
        setArmTarget(angle, null, null) ;
    }

    public void setArmTarget(Angle angle, Angle postol, AngularVelocity veltol) {
        arm_target_ = angle;  
        arm_pos_tolerance_ = postol ;
        arm_vel_tolerance_ = veltol ;
        io_.setArmTarget(angle); 
    }

    public LinearVelocity getElevatorVelocity() {
        return inputs_.elevatorVelocity ;
    }

    public void resetElevator() {
        elevator_calibrated_ = true ;
        io_.setElevatorPosition(Centimeters.zero()) ;
        io_.setArmTarget(ManipulatorConstants.Arm.Positions.kStow);
    }

    public Distance getElevatorPosition() {
        return inputs_.elevatorPosition; 
    }

    public Distance getElevatorTarget() {
        return elev_target_;
    }

    public void setElevatorPosition(Distance d) {
        io_.setElevatorPosition(d);
    }

    public void setElevatorTarget(Distance dist) {
        setElevatorTarget(dist, null, null) ;
    }

    public void setElevatorTarget(Distance dist, Distance postol, LinearVelocity veltol) {
        elev_target_ = dist;
        elev_pos_tolerance_ = postol ;
        elev_vel_tolerance_ = veltol ;
        io_.setElevatorTarget(dist); 
    }

    public void setElevatorVoltage(Voltage volts) {
        io_.setElevatorMotorVoltage(volts.in(Volts)) ;
    }

    public boolean isElevAtTarget() {
        if (elev_target_ == null)
            return false;

        Distance postol = (elev_pos_tolerance_ != null) ? elev_pos_tolerance_ : ManipulatorConstants.Elevator.kPosTolerance ;
        // LinearVelocity veltol = (elev_vel_tolerance_ != null) ? elev_vel_tolerance_ : ManipulatorConstants.Elevator.kVelTolerance ;

        if (!inputs_.elevatorPosition.isNear(elev_target_, postol))
            return false ;

        // if (Robot.isReal() && !inputs_.elevatorVelocity.isNear(MetersPerSecond.of(0.0), veltol))
        //     return false ;

        return true ;
    }

    public boolean isArmAtTarget() {
        if (arm_target_ == null)
            return false;            

        Angle postol = (arm_pos_tolerance_ != null) ? arm_pos_tolerance_ : ManipulatorConstants.Arm.kPosTolerance ;
        AngularVelocity veltol = (arm_vel_tolerance_ != null) ? arm_vel_tolerance_ : ManipulatorConstants.Arm.kVelTolerance ;

        if (!inputs_.armPosition.isNear(arm_target_, postol))
            return false ;

        if (Robot.isReal() && !inputs_.armVelocity.isNear(RotationsPerSecond.of(0), veltol))
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