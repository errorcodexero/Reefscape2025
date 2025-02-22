package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

import static edu.wpi.first.units.Units.*;

import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class ClimberIOHardware implements ClimberIO {
    
    private TalonFX climber_motor_;

    private StatusSignal<Angle> climber_pos_sig_; 
    private StatusSignal<AngularVelocity> climber_vel_sig_; 
    private StatusSignal<Voltage> climber_vol_sig_; 
    private StatusSignal<Current> climber_current_sig_; 

    private DigitalInput attached_switch_one_ ;
    private DigitalInput attached_switch_two_ ;

    public ClimberIOHardware() throws Exception {
        attached_switch_one_ = new DigitalInput(ClimberConstants.kAttachedSensorOne);
        attached_switch_two_ = new DigitalInput(ClimberConstants.kAttachedSensorTwo);

        climber_motor_ = TalonFXFactory.createTalonFX(
            ClimberConstants.Climber.kMotorCANID,
            ClimberConstants.Climber.kInverted,
            ClimberConstants.Climber.kcurrentLimit,
            ClimberConstants.Climber.kCurrentLimitTime
        );

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

        SoftwareLimitSwitchConfigs climberLimitSwitchConfig = new SoftwareLimitSwitchConfigs();
        climberLimitSwitchConfig.ForwardSoftLimitEnable = true;
        climberLimitSwitchConfig.ForwardSoftLimitThreshold = ClimberConstants.Climber.kMaxClimberAngle.times(ClimberConstants.Climber.kGearRatio).in(Rotations);
        climberLimitSwitchConfig.ReverseSoftLimitEnable = true;
        climberLimitSwitchConfig.ReverseSoftLimitThreshold = ClimberConstants.Climber.kMinClimberAngle.times(ClimberConstants.Climber.kGearRatio).in(Rotations);

        TalonFXFactory.checkError(ClimberConstants.Climber.kMotorCANID, "set-climber-PID-values", () -> climber_motor_.getConfigurator().apply(climber_pids));
        TalonFXFactory.checkError(ClimberConstants.Climber.kMotorCANID, "set-climber-MM-values", () -> climber_motor_.getConfigurator().apply(climberMotionMagicConfigs));
        TalonFXFactory.checkError(ClimberConstants.Climber.kMotorCANID, "set-climber-limits", () -> climber_motor_.getConfigurator().apply(climberLimitSwitchConfig)) ;
        
        climber_pos_sig_ = climber_motor_.getPosition();
        climber_vel_sig_ = climber_motor_.getVelocity();
        climber_vol_sig_ = climber_motor_.getSupplyVoltage();
        climber_current_sig_ = climber_motor_.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, climber_pos_sig_, climber_vel_sig_, 
        climber_vol_sig_, climber_current_sig_);
    }

    public void moveClimber(Angle angle) {
        climber_motor_.setControl(new MotionMagicVoltage(angle));
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        inputs.attachedSensorOne = attached_switch_one_.get();
        inputs.attachedSensorTwo = attached_switch_two_.get();

        inputs.climberPosition = climber_pos_sig_.getValue().div(ClimberConstants.Climber.kGearRatio);
        inputs.climberVelocity = climber_vel_sig_.getValue().div(ClimberConstants.Climber.kGearRatio);
        inputs.climberVoltage = climber_vol_sig_.getValue();
        inputs.climberCurrent = climber_current_sig_.getValue();
    }

    public void setClimberPosition(Angle angle) {
        climber_motor_.setControl(new MotionMagicVoltage(angle.times(ClimberConstants.Climber.kGearRatio)));
    }
}
