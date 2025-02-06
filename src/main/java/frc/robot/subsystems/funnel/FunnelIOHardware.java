package frc.robot.subsystems.funnel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.DigitalInterrupt;

public class FunnelIOHardware implements FunnelIO {

    private final TalonFX funnelMotor_;
    private final DigitalInterrupt funnelSensor_;

    private final StatusSignal<Angle> funnelPositionSig;
    private final StatusSignal<AngularVelocity> funnelVelocitySig;
    private final StatusSignal<Voltage> funnelVoltageSig;
    private final StatusSignal<Current> funnelCurrentSig;

    private final Debouncer funnelReadyDebouncer_ = new Debouncer(0.5);

    public FunnelIOHardware() throws Exception {
        funnelMotor_ = TalonFXFactory.createTalonFX(
            FunnelConstants.funnelCanId,
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
    }

    @Override
    public void updateInputs(FunnelInputs inputs) {
        StatusCode funnelStatus = BaseStatusSignal.refreshAll(
            funnelVoltageSig,
            funnelCurrentSig,
            funnelVelocitySig,
            funnelPositionSig
        );

        inputs.funnelReady = funnelReadyDebouncer_.calculate(funnelStatus.isOK());

        inputs.funnelPosition = funnelPositionSig.getValue();
        inputs.funnelCurrent = funnelCurrentSig.getValue();
        inputs.funnelVelocity = funnelVelocitySig.getValue();
        inputs.funnelVoltage = funnelVoltageSig.getValue();

        inputs.coralFunnelSensor = funnelSensor_.getSensor();
        inputs.coralFunnelFallingEdge = funnelSensor_.getFalling();
        inputs.coralFunnelRisingEdge = funnelSensor_.getRising();
    }

    @Override
    public void setPosition(Angle angle) {
        funnelMotor_.setControl(new MotionMagicVoltage(angle));
    }
    
}
