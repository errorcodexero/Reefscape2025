package frc.robot.subsystems.CoralPlacer;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class CoralPlacerIO_HW implements CoralPlacerIO{
    private TalonFX arm_motor_; 
    private TalonFX elevator_motor_; 

    public CoralPlacerIO_HW() {

        arm_motor_ = new TalonFX(CoralPlacerConstants.Arm.kMotorCANID);
        elevator_motor_ = new TalonFX(CoralPlacerConstants.Elevator.kMotorCANID);

        // ARM CONFIGS:     
        Slot0Configs arm_pids = new Slot0Configs();
        arm_pids.kP = CoralPlacerConstants.Arm.PID.kP; 
        arm_pids.kI = CoralPlacerConstants.Arm.PID.kI; 
        arm_pids.kD = CoralPlacerConstants.Arm.PID.kD; 
        arm_pids.kV = CoralPlacerConstants.Arm.PID.kV; 
        arm_pids.kA = CoralPlacerConstants.Arm.PID.kA; 
        arm_pids.kG = CoralPlacerConstants.Arm.PID.kG; 
        arm_pids.kS = CoralPlacerConstants.Arm.PID.kS; 
        arm_motor_.getConfigurator().apply(arm_pids); 

        MotionMagicConfigs armMotionMagicConfigs = new MotionMagicConfigs(); 
        armMotionMagicConfigs.MotionMagicCruiseVelocity = CoralPlacerConstants.Arm.MotionMagic.kMaxVelocity;
        armMotionMagicConfigs.MotionMagicAcceleration = CoralPlacerConstants.Arm.MotionMagic.kMaxAcceleration;
        armMotionMagicConfigs.MotionMagicJerk = CoralPlacerConstants.Arm.MotionMagic.kJerk;
        arm_motor_.getConfigurator().apply(armMotionMagicConfigs);

        // ELEVATOR CONFIGS:
        Slot0Configs elevator_pids = new Slot0Configs();
        elevator_pids.kP = CoralPlacerConstants.Elevator.PID.kP;
        elevator_pids.kI = CoralPlacerConstants.Elevator.PID.kI;
        elevator_pids.kD = CoralPlacerConstants.Elevator.PID.kD;
        elevator_pids.kV = CoralPlacerConstants.Elevator.PID.kV;
        elevator_pids.kA = CoralPlacerConstants.Elevator.PID.kA;
        elevator_pids.kG = CoralPlacerConstants.Elevator.PID.kG;
        elevator_pids.kS = CoralPlacerConstants.Elevator.PID.kS;
        elevator_motor_.getConfigurator().apply(elevator_pids);


        MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = CoralPlacerConstants.Elevator.MotionMagic.kMaxVelocity;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = CoralPlacerConstants.Elevator.MotionMagic.kMaxAcceleration;
        elevatorMotionMagicConfigs.MotionMagicJerk = CoralPlacerConstants.Elevator.MotionMagic.kJerk;
        elevator_motor_.getConfigurator().apply(elevatorMotionMagicConfigs);
    }

    // updates all of the inputs from CoralPlacerIO 
    @Override
    public void updateInputs(CoralPlacerIOInputs inputs) {
        // update all of the necessary inputs here
    }

    public void setArmMotorVoltage(double vol) {
        // code goes here, look at documentation 
    }

    public void logArmMotor(SysIdRoutineLog log) {
        // code goes here, look at documentation 
    }
}
