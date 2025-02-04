package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;



public class ClimberIOHardware implements ClimberIO {
    
    private TalonFX climber_motor_;

    public ClimberIOHardware() {
        climber_motor_ = new TalonFX(ClimberConstants.ClimberArm.kMotorCANID);

        Slot0Configs climber_pids = new Slot0Configs();
        climber_pids.kP = ClimberConstants.ClimberArm.PID.kP;
        climber_pids.kI = ClimberConstants.ClimberArm.PID.kI;
        climber_pids.kD = ClimberConstants.ClimberArm.PID.kD;
        climber_pids.kV = ClimberConstants.ClimberArm.PID.kV;
        climber_pids.kA = ClimberConstants.ClimberArm.PID.kA;
        climber_pids.kG = ClimberConstants.ClimberArm.PID.kG;
        climber_pids.kS = ClimberConstants.ClimberArm.PID.kS;

        MotionMagicConfigs climberMotionMagicConfigs = new MotionMagicConfigs();
        climberMotionMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.ClimberArm.MotionMagic.kMaxVelocity;
        climberMotionMagicConfigs.MotionMagicAcceleration = ClimberConstants.ClimberArm.MotionMagic.kMaxAcceleration;
        climberMotionMagicConfigs.MotionMagicJerk = ClimberConstants.ClimberArm.MotionMagic.kJerk;

    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs_) {
        
    }
        
}

