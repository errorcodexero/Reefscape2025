package frc.robot.subsystems.funnel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.xerosw.util.DigitalInterrupt;
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
import edu.wpi.first.units.measure.Voltage ;

public class FunnelIOHardware implements FunnelIO {
    private final static boolean hasMotor = false ;

    private TalonFX funnelMotor_ = null ;
    private final DigitalInterrupt funnelSensor_;

    private StatusSignal<Angle> funnelPositionSig = null ;
    private StatusSignal<AngularVelocity> funnelVelocitySig = null ;
    private StatusSignal<Voltage> funnelVoltageSig = null ;
    private StatusSignal<Current> funnelCurrentSig = null ;

    private final Debouncer funnelReadyDebouncer_ = new Debouncer(0.5);

    public FunnelIOHardware() throws Exception {
        if (hasMotor) {
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
        }

        funnelSensor_ = new DigitalInterrupt(FunnelConstants.funnelSensorId);
    }

    @Override
    public void updateInputs(FunnelInputs inputs) {
        if (hasMotor) {
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
        }

        inputs.coralFunnelSensor = funnelSensor_.getInput().get();
        inputs.coralFunnelFallingEdge = funnelSensor_.fallingEdge() ;
        inputs.coralFunnelRisingEdge = funnelSensor_.risingEdge() ;
    }

    @Override
    public void setPosition(Angle angle) {
        if (hasMotor) {
            funnelMotor_.setControl(new MotionMagicVoltage(angle));
        }
    }   
}
