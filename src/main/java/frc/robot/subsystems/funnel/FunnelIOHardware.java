package frc.robot.subsystems.funnel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.xerosw.util.DigitalInterrupt;
import org.xerosw.util.EncoderMapper;
import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage ;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;

public class FunnelIOHardware implements FunnelIO {
    private TalonFX funnelMotor_ = null ;
    private final DigitalInterrupt funnelSensor_;
    private DutyCycleEncoder encoder_; 
    private EncoderMapper mapper_; 

    private StatusSignal<Angle> funnelPositionSig = null ;
    private StatusSignal<AngularVelocity> funnelVelocitySig = null ;
    private StatusSignal<Voltage> funnelVoltageSig = null ;
    private StatusSignal<Current> funnelCurrentSig = null ;

    private boolean encoder_motor_synced_ = false ;

    private final Debouncer funnelReadyDebouncer_ = new Debouncer(0.5);

    public FunnelIOHardware() throws Exception {
        encoder_ = new DutyCycleEncoder(FunnelConstants.ThruBoreEncoder.kAbsEncoder);

        mapper_ = new EncoderMapper(
            FunnelConstants.ThruBoreEncoder.kRobotMax,
            FunnelConstants.ThruBoreEncoder.kRobotMin,
            FunnelConstants.ThruBoreEncoder.kEncoderMax,
            FunnelConstants.ThruBoreEncoder.kEncoderMin
        );

        // ENCODER CONFIGS: 
        mapper_.calibrate(FunnelConstants.ThruBoreEncoder.kRobotCalibrationValue,
        FunnelConstants.ThruBoreEncoder.kEncoderCalibrationValue); 

        funnelMotor_ = TalonFXFactory.createTalonFX(
            FunnelConstants.funnelCanId,
            "",
            true,
            FunnelConstants.currentLimit,
            FunnelConstants.lowerTime
        );

        Slot0Configs pids = new Slot0Configs();
        pids.kP = FunnelConstants.MotorPids.kP;
        pids.kI = FunnelConstants.MotorPids.kI;
        pids.kD = FunnelConstants.MotorPids.kD;
        pids.kV = FunnelConstants.MotorPids.kV;
        pids.kA = FunnelConstants.MotorPids.kA;
        pids.kG = FunnelConstants.MotorPids.kG;
        pids.kS = FunnelConstants.MotorPids.kS;
        
        TalonFXFactory.checkError(
            FunnelConstants.funnelCanId,
            "funnel motor pids",
            () -> funnelMotor_.getConfigurator().apply(pids)
        );

        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = FunnelConstants.maxVelocity.in(RotationsPerSecond);
        motionMagicConfig.MotionMagicAcceleration = FunnelConstants.maxAcceleration.in(RotationsPerSecondPerSecond);
        motionMagicConfig.MotionMagicJerk = FunnelConstants.jerk;

        TalonFXFactory.checkError(
            FunnelConstants.funnelCanId,
            "funnel motor mm",
            () -> funnelMotor_.getConfigurator().apply(motionMagicConfig)
        );

        funnelVoltageSig = funnelMotor_.getMotorVoltage();
        funnelCurrentSig = funnelMotor_.getSupplyCurrent();
        funnelVelocitySig = funnelMotor_.getVelocity();
        funnelPositionSig = funnelMotor_.getPosition();

        funnelSensor_ = new DigitalInterrupt(FunnelConstants.funnelSensorId);

        SoftwareLimitSwitchConfigs funnelLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        funnelLimitSwitchConfigs.ForwardSoftLimitEnable = true;
        funnelLimitSwitchConfigs.ForwardSoftLimitThreshold = FunnelConstants.funnelArmMaxAngle.times(FunnelConstants.kGearRatio).in(Rotations) ;
        funnelLimitSwitchConfigs.ReverseSoftLimitEnable = true;
        funnelLimitSwitchConfigs.ReverseSoftLimitThreshold = FunnelConstants.funnelArmMinAngle.times(FunnelConstants.kGearRatio).in(Rotations) ;
    }

    @Override
    public void updateInputs(FunnelInputs inputs) {
        if (!encoder_motor_synced_ && RobotState.isEnabled()) {
            syncFunnelPosition() ;
            encoder_motor_synced_ = true ;
        }
  
        StatusCode funnelStatus = BaseStatusSignal.refreshAll(
            funnelVoltageSig,
            funnelCurrentSig,
            funnelVelocitySig,
            funnelPositionSig
        );

        inputs.funnelReady = funnelReadyDebouncer_.calculate(funnelStatus.isOK());

        inputs.funnelRawPosition = funnelPositionSig.getValue();
        inputs.funnelPosition = inputs.funnelRawPosition.div(FunnelConstants.kGearRatio) ;

        inputs.funnelRawVelocity = funnelVelocitySig.getValue();
        inputs.funnelVelocity = inputs.funnelRawVelocity.div(FunnelConstants.kGearRatio) ;

        inputs.funnelCurrent = funnelCurrentSig.getValue();
        inputs.funnelVoltage = funnelVoltageSig.getValue();

        inputs.coralFunnelSensor = funnelSensor_.getInput().get();
        inputs.coralFunnelFallingEdge = funnelSensor_.fallingEdge() ;
        inputs.coralFunnelRisingEdge = funnelSensor_.risingEdge() ;

        inputs.absEncoderRawValue = encoder_.get();
        inputs.absEncoderValue = Degrees.of(mapper_.toRobot(inputs.absEncoderRawValue)) ;
    }

    @Override
    public void setPosition(Angle angle) {
        funnelMotor_.setControl(new MotionMagicVoltage(angle.times(FunnelConstants.kGearRatio)).withEnableFOC(true)) ;
    }   

    public void syncFunnelPosition() {
        double enc = encoder_.get() ;
        double angle = mapper_.toRobot(enc) ;
        Angle armAngle = Degrees.of(angle).times(FunnelConstants.kGearRatio) ;
        funnelMotor_.setPosition(armAngle) ;
    }    
}
