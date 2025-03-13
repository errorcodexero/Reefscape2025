package frc.robot.subsystems.funnel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;

public class FunnelIOHardware implements FunnelIO {
    private TalonFX funnelMotor_ = null ;
    private final DigitalInput lower_sensor_ ;
    private final DigitalInput upper_sensor_ ;

    private StatusSignal<Angle> funnelPositionSig = null ;
    private StatusSignal<AngularVelocity> funnelVelocitySig = null ;
    private StatusSignal<Voltage> funnelVoltageSig = null ;
    private StatusSignal<Current> funnelCurrentSig = null ;

    private final Debouncer funnelReadyDebouncer_ = new Debouncer(0.5);

    private boolean is_inited_ ;

    public FunnelIOHardware() throws Exception {
        is_inited_ = false ;
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

        lower_sensor_ = new DigitalInput(FunnelConstants.funnelLowerSensorId);
        upper_sensor_ = new DigitalInput(FunnelConstants.funnelUpperSensorId);

        SoftwareLimitSwitchConfigs funnelLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        funnelLimitSwitchConfigs.ForwardSoftLimitEnable = true;
        funnelLimitSwitchConfigs.ForwardSoftLimitThreshold = FunnelConstants.kMaxPosition.in(Rotations) ;
        funnelLimitSwitchConfigs.ReverseSoftLimitEnable = true;
        funnelLimitSwitchConfigs.ReverseSoftLimitThreshold = FunnelConstants.kMinPosition.in(Rotations) ;

        funnelMotor_.setPosition(0.0) ;
        funnelMotor_.setControl(new MotionMagicVoltage(Degrees.of(0.0))) ;
    }

    @Override
    public void updateInputs(FunnelInputs inputs) {

        if (RobotState.isAutonomous() && RobotState.isEnabled() && !is_inited_) {
            funnelMotor_.setPosition(0.0) ;
            is_inited_ = true ;
        }

        StatusCode funnelStatus = BaseStatusSignal.refreshAll(
            funnelVoltageSig,
            funnelCurrentSig,
            funnelVelocitySig,
            funnelPositionSig
        );

        inputs.funnelReady = funnelReadyDebouncer_.calculate(funnelStatus.isOK());

        inputs.funnelPosition = funnelPositionSig.getValue() ;
        inputs.funnelVelocity = funnelVelocitySig.getValue();

        inputs.funnelCurrent = funnelCurrentSig.getValue();
        inputs.funnelVoltage = funnelVoltageSig.getValue();

        inputs.coralFunnelUpperSensor = !upper_sensor_.get() ;
        inputs.coralFunnelLowerSensor = !lower_sensor_.get() ;
    }

    @Override
    public void setTargetPosition(Angle v) {
        funnelMotor_.setControl(new MotionMagicVoltage(v).withEnableFOC(true)) ;
    }   

    public void setVoltage(Voltage volts) {
        funnelMotor_.setVoltage(volts.in(Volts)) ;
    }
}
