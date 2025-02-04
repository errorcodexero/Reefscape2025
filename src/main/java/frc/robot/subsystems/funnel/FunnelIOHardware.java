package frc.robot.subsystems.funnel;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import static edu.wpi.first.units.Units.Volts;
import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class FunnelIOHardware implements FunnelIO {
    private TalonFX motor_;
    private Voltage volts_ ;

    private StatusSignal<Angle> position_ ;
    private StatusSignal<AngularVelocity> velocity_ ;
    private StatusSignal<Current> current_ ;
    private StatusSignal<Voltage> voltage_ ;

    public FunnelIOHardware() {
        createFunnel();
    }

    public void updateInputs(FunnelIOInputs inputs) {
        inputs.funnelPosition = position_.refresh().getValue().times(FunnelConstants.Funnel.kGearRatio) ;
        inputs.funnelVelocity = velocity_.refresh().getValue().times(FunnelConstants.Funnel.kGearRatio) ;
        inputs.funnelCurrent = current_.refresh().getValue() ;
        inputs.funnelVoltage = voltage_.refresh().getValue() ;
    }

    public void setFunnelPosition(Angle target) {
        ControlRequest ctrl = new MotionMagicVoltage(target.div(FunnelConstants.Funnel.kGearRatio)) ;
        motor_.setControl(ctrl) ;
    }

    public void setFunnelMotorVoltage(double volts) {
        volts_ = Volts.of(volts) ;
        ControlRequest ctrl = new VoltageOut(volts_) ;
        motor_.setControl(ctrl) ;
    }

    public void logFunnelMotor(SysIdRoutineLog log) {
        Angle pos = position_.refresh().getValue().times(FunnelConstants.Funnel.kGearRatio) ;
        AngularVelocity vel = velocity_.refresh().getValue().times(FunnelConstants.Funnel.kGearRatio) ;

        log.motor("roller")
            .voltage(volts_)
            .angularPosition(pos)
            .angularVelocity(vel) ;        
    }

    private void createFunnel() {
        TalonFXFactory f = TalonFXFactory.getFactory() ;
        motor_ = f.createTalonFX(FunnelConstants.Funnel.kMotorCANID, 
                                         FunnelConstants.Funnel.kInverted,
                                         FunnelConstants.Funnel.kCurrentLimit) ;

        position_ = motor_.getPosition() ;
        velocity_ = motor_.getVelocity() ;
        current_ = motor_.getSupplyCurrent() ;
        voltage_ = motor_.getMotorVoltage() ;        

        // ELEVATOR CONFIGS:
        Slot0Configs pids = new Slot0Configs();
        pids.kP = FunnelConstants.Funnel.PID.kP;
        pids.kI = FunnelConstants.Funnel.PID.kI;
        pids.kD = FunnelConstants.Funnel.PID.kD;
        pids.kV = FunnelConstants.Funnel.PID.kV;
        pids.kA = FunnelConstants.Funnel.PID.kA;
        pids.kG = FunnelConstants.Funnel.PID.kG;
        pids.kS = FunnelConstants.Funnel.PID.kS;
        TalonFXFactory.checkError(FunnelConstants.Funnel.kMotorCANID, "apply", () -> motor_.getConfigurator().apply(pids)) ;

        MotionMagicConfigs rollerMotionMagicConfigs = new MotionMagicConfigs();
        rollerMotionMagicConfigs.MotionMagicCruiseVelocity = FunnelConstants.Funnel.MotionMagic.kMaxVelocity;
        rollerMotionMagicConfigs.MotionMagicAcceleration = FunnelConstants.Funnel.MotionMagic.kMaxAcceleration;
        rollerMotionMagicConfigs.MotionMagicJerk = FunnelConstants.Funnel.MotionMagic.kJerk;
        TalonFXFactory.checkError(FunnelConstants.Funnel.kMotorCANID, "apply", () -> motor_.getConfigurator().apply(rollerMotionMagicConfigs)) ;
    } 
}
