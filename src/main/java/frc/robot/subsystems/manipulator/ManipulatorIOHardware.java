package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class ManipulatorIOHardware implements ManipulatorIO {
    private TalonFX arm_motor_; 
    private TalonFX elevator_motor_front_; 
    private TalonFX elevator_motor_back_; 


    public ManipulatorIOHardware() {

        arm_motor_ = new TalonFX(ManipulatorConstants.Arm.kMotorCANID);
        elevator_motor_front_ = new TalonFX(ManipulatorConstants.Elevator.Front.kMotorCANID);
        elevator_motor_back_ = new TalonFX(ManipulatorConstants.Elevator.Back.kMotorCANID);

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

        // ELEVATOR CONFIGS:
        Slot0Configs elevator_pids = new Slot0Configs();
        elevator_pids.kP = ManipulatorConstants.Elevator.PID.kP;
        elevator_pids.kI = ManipulatorConstants.Elevator.PID.kI;
        elevator_pids.kD = ManipulatorConstants.Elevator.PID.kD;
        elevator_pids.kV = ManipulatorConstants.Elevator.PID.kV;
        elevator_pids.kA = ManipulatorConstants.Elevator.PID.kA;
        elevator_pids.kG = ManipulatorConstants.Elevator.PID.kG;
        elevator_pids.kS = ManipulatorConstants.Elevator.PID.kS;
        elevator_motor_front_.getConfigurator().apply(elevator_pids);
        elevator_motor_back_.getConfigurator().apply(elevator_pids);


        MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = ManipulatorConstants.Elevator.MotionMagic.kMaxVelocity;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = ManipulatorConstants.Elevator.MotionMagic.kMaxAcceleration;
        elevatorMotionMagicConfigs.MotionMagicJerk = ManipulatorConstants.Elevator.MotionMagic.kJerk;
        elevator_motor_front_.getConfigurator().apply(elevatorMotionMagicConfigs);
        elevator_motor_back_.getConfigurator().apply(elevatorMotionMagicConfigs);

    }

    // updates all of the inputs from ManipulatorIO 
    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        // update all of the necessary inputs here
    }

    public void setArmMotorVoltage(double vol) {
        // code goes here, look at documentation 
    }

    public void logArmMotor(SysIdRoutineLog log) {
        // code goes here, look at documentation 
    }

    public void setElevatorPosition(double m){
        elevator_motor_front_.setControl(new MotionMagicVoltage(m / ManipulatorConstants.Elevator.kGearRatio).withSlot(0));
        elevator_motor_back_.setControl(new MotionMagicVoltage(m / ManipulatorConstants.Elevator.kGearRatio).withSlot(0));
    } 

    public double getElevatorFrontPosition(){
        return elevator_motor_front_.getPosition().getValueAsDouble() * ManipulatorConstants.Elevator.kGearRatio; 
    }
 
    public double getElevatorBackPosition(){
        return elevator_motor_back_.getPosition().getValueAsDouble() * ManipulatorConstants.Elevator.kGearRatio; 
    }

    public void setArmPosition(double deg){
        arm_motor_.setControl(new MotionMagicVoltage(deg / ManipulatorConstants.Arm.kGearRatio).withSlot(0)); 
    } 
    
    public double getArmPosition(){
        return arm_motor_.getPosition().getValueAsDouble() * ManipulatorConstants.Arm.kGearRatio;
    }
}