package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Revolution;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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

    public ManipulatorIOHardware() {

        arm_motor_ = new TalonFX(ManipulatorConstants.Arm.kMotorCANID);
        elevator_motor_ = new TalonFX(ManipulatorConstants.Elevator.kMotorCANID);

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

        MotionMagicConfigs armMotionMagicConfigs = new MotionMagicConfigs(); 
        armMotionMagicConfigs.MotionMagicCruiseVelocity = ManipulatorConstants.Arm.MotionMagic.kMaxVelocity;
        armMotionMagicConfigs.MotionMagicAcceleration = ManipulatorConstants.Arm.MotionMagic.kMaxAcceleration;
        armMotionMagicConfigs.MotionMagicJerk = ManipulatorConstants.Arm.MotionMagic.kJerk;
        arm_motor_.getConfigurator().apply(armMotionMagicConfigs);

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
        elevator_motor_.getConfigurator().apply(elevator_pids);

        MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = ManipulatorConstants.Elevator.MotionMagic.kMaxVelocity;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = ManipulatorConstants.Elevator.MotionMagic.kMaxAcceleration;
        elevatorMotionMagicConfigs.MotionMagicJerk = ManipulatorConstants.Elevator.MotionMagic.kJerk;
        elevator_motor_.getConfigurator().apply(elevatorMotionMagicConfigs);

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
        inputs.armPosition = arm_pos_sig_.refresh().getValue().times(ManipulatorConstants.Arm.kGearRatio);
        inputs.armVelocity = arm_vel_sig_.refresh().getValue().times(ManipulatorConstants.Arm.kGearRatio);
        inputs.armVoltage = arm_vol_sig_.refresh().getValue();
        inputs.armCurrent = arm_current_sig_.refresh().getValue();
        
        double rev = elevator_pos_sig_.refresh().getValue().in(Revolution); 
        inputs.elevatorPosition = Meters.of(rev * ManipulatorConstants.Elevator.kMetersPerRev);

        inputs.elevatorVelocity = elevator_vel_sig_.refresh().getValue().times(ManipulatorConstants.Elevator.kMetersPerRev);
        inputs.elevatorVoltage = elevator_vol_sig_.refresh().getValue();
        inputs.elevatorCurrent = elevator_current_sig_.refresh().getValue();
    }

    public void setArmMotorVoltage(double vol) {
        // code goes here, look at documentation 
    }

    public void logArmMotor(SysIdRoutineLog log) {
        // code goes here, look at documentation 
    }

    public void setElevatorPosition(double m){
        elevator_motor_.setControl(new MotionMagicVoltage(m / ManipulatorConstants.Elevator.kMetersPerRev).withSlot(0));
        elevator_motor_.setControl(new MotionMagicVoltage(m / ManipulatorConstants.Elevator.kMetersPerRev).withSlot(0));
    } 

    public double getElevatorFrontPosition(){
        return elevator_motor_.getPosition().getValueAsDouble() * ManipulatorConstants.Elevator.kMetersPerRev; 
    }
 
    public double getElevatorBackPosition(){
        return elevator_motor_.getPosition().getValueAsDouble() * ManipulatorConstants.Elevator.kMetersPerRev; 
    }

    public void setArmPosition(double deg){
        arm_motor_.setControl(new MotionMagicVoltage(deg / ManipulatorConstants.Arm.kGearRatio).withSlot(0)); 
    } 
    
    public double getArmPosition(){
        return arm_motor_.getPosition().getValueAsDouble() * ManipulatorConstants.Arm.kGearRatio;
    }
}