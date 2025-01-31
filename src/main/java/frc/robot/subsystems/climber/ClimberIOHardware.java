package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;



public class ClimberIOHardware implements ClimberIO {
    private TalonFX climber_motor_;

    private StatusSignal<Angle> climber_pos_sig_; 
    private StatusSignal<AngularVelocity> climber_vel_sig_; 
    private StatusSignal<Voltage> climber_vol_sig_; 
    private StatusSignal<Current> climber_current_sig_; 

    public ClimberIOHardware() {
        climber_motor_ = new TalonFX(ClimberConstants.Climber.kMotorCANID);

        Slot0Configs climber_pids = new Slot0Configs();
        climber_pids.kP = ClimberConstants.Climber.PID.kP;
        climber_pids.kI = ClimberConstants.Climber.PID.kI;
        climber_pids.kD = ClimberConstants.Climber.PID.kD;
        climber_pids.kV = ClimberConstants.Climber.PID.kV;
        climber_pids.kA = ClimberConstants.Climber.PID.kA;
        climber_pids.kG = ClimberConstants.Climber.PID.kG;
        climber_pids.kS = ClimberConstants.Climber.PID.kS;

        MotionMagicConfigs climberMotionMagicConfigs = new MotionMagicConfigs();
        climberMotionMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.Climber.MotionMagic.kMaxVelocity;
        climberMotionMagicConfigs.MotionMagicAcceleration = ClimberConstants.Climber.MotionMagic.kMaxAcceleration;
        climberMotionMagicConfigs.MotionMagicJerk = ClimberConstants.Climber.MotionMagic.kJerk;

        climber_pos_sig_ = climber_motor_.getPosition();
        climber_vel_sig_ = climber_motor_.getVelocity();
        climber_vol_sig_ = climber_motor_.getSupplyVoltage();
        climber_current_sig_ = climber_motor_.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, climber_pos_sig_, climber_vel_sig_, 
        climber_vol_sig_, climber_current_sig_);

    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        inputs.climberPosition = climber_pos_sig_.getValue();
        inputs.climberVelocity = climber_vel_sig_.getValue();
        inputs.climberVoltage = climber_vol_sig_.getValue();
        inputs.climberCurrent = climber_current_sig_.getValue();
    }
        
}

