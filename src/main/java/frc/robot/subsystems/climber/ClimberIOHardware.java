package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

import static edu.wpi.first.units.Units.*;

import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class ClimberIOHardware implements ClimberIO {

    private static int kSyncWaitCount = 20 ;
    
    protected TalonFX climber_motor_;
    protected DutyCycleEncoder encoder_; 
    private int sync_count_ ;

    private StatusSignal<Angle> climber_pos_sig_; 
    private StatusSignal<AngularVelocity> climber_vel_sig_; 
    private StatusSignal<Voltage> climber_vol_sig_; 
    private StatusSignal<Current> climber_current_sig_; 

    private DigitalInput attached_switch_one_ ;

    public ClimberIOHardware() throws Exception {
        encoder_ = new DutyCycleEncoder(ClimberConstants.ThruBoreEncoder.kAbsEncoder) ;

        sync_count_ = kSyncWaitCount ;

        attached_switch_one_ = new DigitalInput(ClimberConstants.kAttachedSensor);

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

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {

        if (sync_count_ >= 0) {
            sync_count_-- ;

            if (sync_count_ == 0) {
                syncClimberPosition();
            }
        }

        inputs.syncCount = sync_count_ ;
        inputs.attachedSensor = !attached_switch_one_.get();

        inputs.absEncoderRawValue = encoder_.get();
        inputs.absEncoderValue = Degrees.of(encoderToRobot(inputs.absEncoderRawValue));

        inputs.climberRawPosition = climber_pos_sig_.refresh().getValue();
        inputs.climberPosition = inputs.climberRawPosition.div(ClimberConstants.Climber.kGearRatio);
        inputs.climberVelocity = climber_vel_sig_.refresh().getValue().div(ClimberConstants.Climber.kGearRatio);
        inputs.climberVoltage = climber_vol_sig_.refresh().getValue();
        inputs.climberCurrent = climber_current_sig_.refresh().getValue();
    }

    public void setClimberPosition(Angle angle) {
        climber_motor_.setControl(new MotionMagicVoltage(angle.times(ClimberConstants.Climber.kGearRatio)).withEnableFOC(true)) ;
    }

    public void syncClimberPosition() {
        double enc = encoder_.get() ;
        double angle = encoderToRobot(enc) ;
        Angle armAngle = Degrees.of(angle).times(ClimberConstants.Climber.kGearRatio) ;
        climber_motor_.setPosition(armAngle) ;
    }

    public void setClimberVoltage(Voltage voltage) {
        climber_motor_.setControl(new VoltageOut(voltage)) ;
    }

    protected double encoderToRobot(double encoderValue) {
        if (encoderValue < ClimberConstants.kMinAbsEncoderRollover) {
            encoderValue += 1.0 ;
        }
        
        return encoderValue * ClimberConstants.kMConvertAbsToRobot + ClimberConstants.kBConvertAbsToRobot;
    }

    protected double robotToEncoder(double robotValue) {
        double encoderValue = (robotValue - ClimberConstants.kBConvertAbsToRobot) / ClimberConstants.kMConvertAbsToRobot;
    
        if (encoderValue >= ClimberConstants.kMinAbsEncoderRollover + 1.0) {
            encoderValue -= 1.0;
        }
    
        return encoderValue;
    }
}
