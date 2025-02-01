package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Revolution;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class ManipulatorIOHardware implements ManipulatorIO {
    private TalonFX arm_motor_; 
    private TalonFX elevator_motor_; 

    private StatusSignal<Angle> arm_pos_sig_; 
    private StatusSignal<AngularVelocity> arm_vel_sig_; 
    private StatusSignal<Voltage> arm_vol_sig_; 
    private StatusSignal<Current> arm_current_sig_; 

    private StatusSignal<Angle> elevator_pos_sig_; 
    private StatusSignal<AngularVelocity> elevator_vel_sig_; 
    private StatusSignal<Voltage> elevator_vol_sig_; 
    private StatusSignal<Current> elevator_current_sig_;

    private Voltage arm_voltage_; 
    private Voltage elevator_voltage_; 

    public ManipulatorIOHardware() throws Exception{

        arm_motor_ = TalonFXFactory.createTalonFX(ManipulatorConstants.Arm.kMotorCANID, ManipulatorConstants.Arm.kCANBusName, ManipulatorConstants.Arm.kInverted); 
        elevator_motor_ = TalonFXFactory.createTalonFX(ManipulatorConstants.Elevator.kMotorCANID, ManipulatorConstants.Elevator.kCANBusName, ManipulatorConstants.Elevator.kInverted); 

        // ARM CONFIGS: 
        Slot0Configs arm_pids = new Slot0Configs();
        arm_pids.kP = ManipulatorConstants.Arm.PID.kP; 
        arm_pids.kI = ManipulatorConstants.Arm.PID.kI; 
        arm_pids.kD = ManipulatorConstants.Arm.PID.kD; 
        arm_pids.kV = ManipulatorConstants.Arm.PID.kV; 
        arm_pids.kA = ManipulatorConstants.Arm.PID.kA; 
        arm_pids.kG = ManipulatorConstants.Arm.PID.kG; 
        arm_pids.kS = ManipulatorConstants.Arm.PID.kS; 
        arm_motor_.getConfigurator().apply(arm_pids); 
        TalonFXFactory.checkError(ManipulatorConstants.Arm.kMotorCANID, "set-arm-PID-values", () -> arm_motor_.getConfigurator().apply(arm_pids)); 

        MotionMagicConfigs armMotionMagicConfigs = new MotionMagicConfigs(); 
        armMotionMagicConfigs.MotionMagicCruiseVelocity = ManipulatorConstants.Arm.MotionMagic.kMaxVelocity;
        armMotionMagicConfigs.MotionMagicAcceleration = ManipulatorConstants.Arm.MotionMagic.kMaxAcceleration;
        armMotionMagicConfigs.MotionMagicJerk = ManipulatorConstants.Arm.MotionMagic.kJerk;
        TalonFXFactory.checkError(ManipulatorConstants.Arm.kMotorCANID, "set-arm-MM-values", () -> arm_motor_.getConfigurator().apply(armMotionMagicConfigs)); 

        arm_pos_sig_ = arm_motor_.getPosition();
        arm_vel_sig_ = arm_motor_.getVelocity();
        arm_vol_sig_ = arm_motor_.getSupplyVoltage();
        arm_current_sig_ = arm_motor_.getSupplyCurrent();

        // ELEVATOR CONFIGS:
        Slot0Configs elevator_pids = new Slot0Configs();
        elevator_pids.kP = ManipulatorConstants.Elevator.PID.kP;
        elevator_pids.kI = ManipulatorConstants.Elevator.PID.kI;
        elevator_pids.kD = ManipulatorConstants.Elevator.PID.kD;
        elevator_pids.kV = ManipulatorConstants.Elevator.PID.kV;
        elevator_pids.kA = ManipulatorConstants.Elevator.PID.kA;
        elevator_pids.kG = ManipulatorConstants.Elevator.PID.kG;
        elevator_pids.kS = ManipulatorConstants.Elevator.PID.kS;
        TalonFXFactory.checkError(ManipulatorConstants.Elevator.kMotorCANID, "set-elevator-MM-values", () -> elevator_motor_.getConfigurator().apply(elevator_pids)); 

        MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = ManipulatorConstants.Elevator.MotionMagic.kMaxVelocity;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = ManipulatorConstants.Elevator.MotionMagic.kMaxAcceleration;
        elevatorMotionMagicConfigs.MotionMagicJerk = ManipulatorConstants.Elevator.MotionMagic.kJerk;
        TalonFXFactory.checkError(ManipulatorConstants.Elevator.kMotorCANID, "set-elevator-MM-values", () -> elevator_motor_.getConfigurator().apply(elevatorMotionMagicConfigs)); 

        elevator_pos_sig_ = elevator_motor_.getPosition();
        elevator_vel_sig_ = elevator_motor_.getVelocity();
        elevator_vol_sig_ = elevator_motor_.getSupplyVoltage();
        elevator_current_sig_ = elevator_motor_.getSupplyCurrent();

        // setting signal update frequency: 
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, arm_pos_sig_, arm_vel_sig_, 
            arm_vol_sig_, arm_current_sig_, elevator_pos_sig_, elevator_vel_sig_, elevator_vol_sig_,
            elevator_current_sig_); 
    }

    // updates all of the inputs from ManipulatorIO 
    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        // arm inputs: 
        inputs.armPosition = arm_pos_sig_.refresh().getValue().times(ManipulatorConstants.Arm.kGearRatio);
        inputs.armVelocity = arm_vel_sig_.refresh().getValue().times(ManipulatorConstants.Arm.kGearRatio);
        inputs.armVoltage = arm_vol_sig_.refresh().getValue();
        inputs.armCurrent = arm_current_sig_.refresh().getValue();
        
        // elevator inputs: 
        double rev = elevator_pos_sig_.refresh().getValue().in(Revolution); 
        inputs.elevatorPosition = Meters.of(rev * ManipulatorConstants.Elevator.kMetersPerRev);

        double vel = elevator_vel_sig_.refresh().getValue().in(DegreesPerSecond); 
        inputs.elevatorVelocity = MetersPerSecond.of(vel * ManipulatorConstants.Elevator.kMetersPerRev); 

        inputs.elevatorVoltage = elevator_vol_sig_.refresh().getValue();
        inputs.elevatorCurrent = elevator_current_sig_.refresh().getValue();
    }

    public void setArmMotorVoltage(Voltage vol) {
        arm_voltage_ = vol;
        arm_motor_.setControl(new VoltageOut(arm_voltage_));
    }

    public void logArmMotor(SysIdRoutineLog log) {
        log.motor("arm")
            .voltage(arm_voltage_)
            .angularPosition(Revolutions.of(arm_pos_sig_.refresh().getValueAsDouble()))
            .angularVelocity(RevolutionsPerSecond.of(arm_vel_sig_.refresh().getValueAsDouble()));
    }

    public void setElevatorMotorVoltage(Voltage vol) {
        elevator_voltage_ = vol;
        elevator_motor_.setControl(new VoltageOut(elevator_voltage_));    
    }

    public void logElevatorMotor(SysIdRoutineLog log) {
        log.motor("elevator")
            .voltage(elevator_voltage_)
            .linearVelocity(MetersPerSecond.of(elevator_vel_sig_.refresh().getValueAsDouble()));    
        } 

    public void setElevatorPosition(Distance dist) {
        double revs = dist.in(Meters) / ManipulatorConstants.Elevator.kMetersPerRev ;
        elevator_motor_.setControl(new MotionMagicVoltage(Revolutions.of(revs)).withSlot(0));
    }

    public void setArmPosition(Angle angle) {
        arm_motor_.setControl(new MotionMagicVoltage(angle.div(ManipulatorConstants.Arm.kGearRatio)).withSlot(0)); 
    }
}