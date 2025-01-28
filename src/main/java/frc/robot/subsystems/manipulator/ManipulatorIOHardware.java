package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class ManipulatorIOHardware implements ManipulatorIO {

    // Arm related members
    private TalonFX arm_motor_;
    private Voltage arm_volts_ ;
    private StatusSignal<Angle> arm_position_ ;
    private StatusSignal<AngularVelocity> arm_velocity_ ;
    private StatusSignal<Current> arm_current_ ;
    private StatusSignal<Voltage> arm_voltage_ ;

    // Elevator related members
    private TalonFX elevator_motor_; 
    private Voltage elevator_volts_ ;
    private StatusSignal<Angle> elevator_position_ ;
    private StatusSignal<AngularVelocity> elevator_velocity_ ;
    private StatusSignal<Current> elevator_current_ ;
    private StatusSignal<Voltage> elevator_voltage_ ;


    public ManipulatorIOHardware() {
        createArm();
        createElevator();

        TalonFXFactory.checkError(-1, "manipulator-update-frequencies-50", () -> BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50.0),
                        arm_position_,
                        arm_velocity_,
                        arm_current_,
                        arm_voltage_,
                        elevator_position_,
                        elevator_velocity_,
                        elevator_current_,
                        elevator_voltage_)) ;



        arm_motor_.optimizeBusUtilization() ;
        elevator_motor_.optimizeBusUtilization() ;

    }

    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.armPosition = arm_position_.refresh().getValue().times(ManipulatorConstants.Arm.kGearRatio) ;
        inputs.armVelocity = arm_velocity_.refresh().getValue().times(ManipulatorConstants.Arm.kGearRatio) ;
        inputs.armCurrent = arm_current_.refresh().getValue() ;
        inputs.armVoltage = arm_voltage_.refresh().getValue() ;

        inputs.elevatorPosition = Meters.of(elevator_position_.refresh().getValue().in(Revolutions) * ManipulatorConstants.Elevator.kMotorRevsToHeightMeters) ;
        inputs.elevatorVelocity = MetersPerSecond.of(elevator_velocity_.refresh().getValue().in(RevolutionsPerSecond) * ManipulatorConstants.Elevator.kMotorRevsToHeightMeters) ;
        inputs.elevatorCurrent = elevator_current_.refresh().getValue() ;
        inputs.elevatorVoltage = elevator_voltage_.refresh().getValue() ;
    }

    // arm related methods
    public void setArmAngle(Angle target) {
        ControlRequest ctrl = new MotionMagicVoltage(target.div(ManipulatorConstants.Arm.kGearRatio)).withSlot(0).withEnableFOC(true) ;
        arm_motor_.setControl(ctrl) ;
    }

    public void setArmMotorVoltage(double volts) {
        arm_volts_ = Volts.of(volts) ;
        ControlRequest ctrl = new VoltageOut(arm_volts_) ;
        arm_motor_.setControl(ctrl) ;
    }

    public void logArmMotor(SysIdRoutineLog log) {
        Angle pos = arm_position_.refresh().getValue().times(ManipulatorConstants.Arm.kGearRatio) ;
        AngularVelocity vel = arm_velocity_.refresh().getValue().times(ManipulatorConstants.Arm.kGearRatio) ;

        log.motor("arm")
            .voltage(arm_volts_)
            .angularPosition(pos)
            .angularVelocity(vel) ;
    }

    // elevator related methods
    public void setElevatorHeight(Distance target) {
        Angle pos = Revolutions.of(target.in(Meters) / ManipulatorConstants.Elevator.kMotorRevsToHeightMeters) ;
        ControlRequest ctrl = new MotionMagicVoltage(pos).withSlot(0).withEnableFOC(true) ;
        elevator_motor_.setControl(ctrl) ;
    }

    public void setElevatorMotorVoltage(double volts) {
        elevator_volts_ = Volts.of(volts) ;
        ControlRequest ctrl = new VoltageOut(elevator_volts_) ;
        elevator_motor_.setControl(ctrl) ;
    }

    public void logElevatorMotor(SysIdRoutineLog log) {
        Distance pos = Meters.of(elevator_position_.refresh().getValue().in(Revolutions) * ManipulatorConstants.Elevator.kMotorRevsToHeightMeters) ;
        LinearVelocity vel = MetersPerSecond.of(elevator_velocity_.refresh().getValue().in(RevolutionsPerSecond) * ManipulatorConstants.Elevator.kMotorRevsToHeightMeters) ;

        log.motor("arm")
            .voltage(arm_volts_)
            .linearPosition(pos)
            .linearVelocity(vel) ;
    }

    private void createArm() {
        TalonFXFactory f = TalonFXFactory.getFactory() ;
        arm_motor_ = f.createTalonFX(ManipulatorConstants.Arm.kMotorCANID, ManipulatorConstants.Arm.kInverted) ;

        arm_position_ = arm_motor_.getPosition() ;
        arm_velocity_ = arm_motor_.getVelocity() ;
        arm_current_ = arm_motor_.getSupplyCurrent() ;
        arm_voltage_ = arm_motor_.getMotorVoltage() ;

        // ARM CONFIGS:     
        Slot0Configs arm_pids = new Slot0Configs();
        arm_pids.kP = ManipulatorConstants.Arm.PID.kP; 
        arm_pids.kI = ManipulatorConstants.Arm.PID.kI; 
        arm_pids.kD = ManipulatorConstants.Arm.PID.kD; 
        arm_pids.kV = ManipulatorConstants.Arm.PID.kV; 
        arm_pids.kA = ManipulatorConstants.Arm.PID.kA; 
        arm_pids.kG = ManipulatorConstants.Arm.PID.kG; 
        arm_pids.kS = ManipulatorConstants.Arm.PID.kS; 
        TalonFXFactory.checkError(ManipulatorConstants.Arm.kMotorCANID, "apply", () -> arm_motor_.getConfigurator().apply(arm_pids)) ;

        MotionMagicConfigs armMotionMagicConfigs = new MotionMagicConfigs(); 
        armMotionMagicConfigs.MotionMagicCruiseVelocity = ManipulatorConstants.Arm.MotionMagic.kMaxVelocity;
        armMotionMagicConfigs.MotionMagicAcceleration = ManipulatorConstants.Arm.MotionMagic.kMaxAcceleration;
        armMotionMagicConfigs.MotionMagicJerk = ManipulatorConstants.Arm.MotionMagic.kJerk;
        TalonFXFactory.checkError(ManipulatorConstants.Arm.kMotorCANID, "apply", () -> arm_motor_.getConfigurator().apply(armMotionMagicConfigs)) ;
    }

    private void createElevator() {
        TalonFXFactory f = TalonFXFactory.getFactory() ;
        elevator_motor_ = f.createTalonFX(ManipulatorConstants.Elevator.kMotorCANID, ManipulatorConstants.Elevator.kInverted) ;

        elevator_position_ = elevator_motor_.getPosition() ;
        elevator_velocity_ = elevator_motor_.getVelocity() ;
        elevator_current_ = elevator_motor_.getSupplyCurrent() ;
        elevator_voltage_ = elevator_motor_.getMotorVoltage() ;

        // ELEVATOR CONFIGS:
        Slot0Configs elevator_pids = new Slot0Configs();
        elevator_pids.kP = ManipulatorConstants.Elevator.PID.kP;
        elevator_pids.kI = ManipulatorConstants.Elevator.PID.kI;
        elevator_pids.kD = ManipulatorConstants.Elevator.PID.kD;
        elevator_pids.kV = ManipulatorConstants.Elevator.PID.kV;
        elevator_pids.kA = ManipulatorConstants.Elevator.PID.kA;
        elevator_pids.kG = ManipulatorConstants.Elevator.PID.kG;
        elevator_pids.kS = ManipulatorConstants.Elevator.PID.kS;
        TalonFXFactory.checkError(ManipulatorConstants.Elevator.kMotorCANID, "apply", () -> elevator_motor_.getConfigurator().apply(elevator_pids)) ;

        MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = ManipulatorConstants.Elevator.MotionMagic.kMaxVelocity;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = ManipulatorConstants.Elevator.MotionMagic.kMaxAcceleration;
        elevatorMotionMagicConfigs.MotionMagicJerk = ManipulatorConstants.Elevator.MotionMagic.kJerk;
        TalonFXFactory.checkError(ManipulatorConstants.Elevator.kMotorCANID, "apply", () -> elevator_motor_.getConfigurator().apply(elevatorMotionMagicConfigs)) ;
    }

}