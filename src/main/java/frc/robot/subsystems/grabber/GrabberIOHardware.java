package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.*;

import org.xerosw.util.DigitalInterrupt;
import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class GrabberIOHardware implements GrabberIO {
    private DigitalInterrupt coral_;
    private DigitalInterrupt algae_;

    private TalonFX grabber_motor_;

    private double grabber_voltage_;

    private StatusSignal<Angle> grabber_pos_sig_;
    private StatusSignal<AngularVelocity> grabber_vel_sig_;
    private StatusSignal<Current> grabber_curr_sig_;
    private StatusSignal<Voltage> grabber_vol_sig_;

    private final Debouncer grabberErrorDebounce_ = new Debouncer(0.5);

    public GrabberIOHardware() throws Exception {

        grabber_motor_ = TalonFXFactory.createTalonFX(
            GrabberConstants.Grabber.kMotorCANID,
            null, GrabberConstants.Grabber.kInverted,
            Amps.of(40),
            Seconds.of(1)
        );

        grabber_pos_sig_ = grabber_motor_.getPosition() ;
        grabber_vel_sig_ = grabber_motor_.getVelocity() ;
        grabber_curr_sig_ = grabber_motor_.getSupplyCurrent() ;
        grabber_vol_sig_ = grabber_motor_.getMotorVoltage() ;

        Slot0Configs grabber_pids_ = new Slot0Configs();
        grabber_pids_.kP = GrabberConstants.Grabber.PID.kP;
        grabber_pids_.kI = GrabberConstants.Grabber.PID.kI;
        grabber_pids_.kD = GrabberConstants.Grabber.PID.kD;
        grabber_pids_.kV = GrabberConstants.Grabber.PID.kV;
        grabber_pids_.kA = GrabberConstants.Grabber.PID.kA;
        grabber_pids_.kG = GrabberConstants.Grabber.PID.kG;
        grabber_pids_.kS = GrabberConstants.Grabber.PID.kS;
        
        MotionMagicConfigs grabberMotionMagicConfigs = new MotionMagicConfigs();
        grabberMotionMagicConfigs.MotionMagicCruiseVelocity = GrabberConstants.Grabber.MotionMagic.kMaxVelocity;
        grabberMotionMagicConfigs.MotionMagicAcceleration = GrabberConstants.Grabber.MotionMagic.kMaxAcceleration;
        grabberMotionMagicConfigs.MotionMagicJerk = GrabberConstants.Grabber.MotionMagic.kJerk;

        TalonFXFactory.checkError(GrabberConstants.Grabber.kMotorCANID, null, () -> grabber_motor_.getConfigurator().apply(grabber_pids_));

        TalonFXFactory.checkError(0, null, () -> grabber_motor_.getConfigurator().apply(grabberMotionMagicConfigs), 5);

        TalonFXFactory.checkError(
            GrabberConstants.Grabber.kMotorCANID,
            "grabber-set-freq",
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                1.0,
                grabber_curr_sig_,
                grabber_pos_sig_,
                grabber_vel_sig_,
                grabber_vol_sig_
            )
        );

        TalonFXFactory.checkError(GrabberConstants.Grabber.kMotorCANID, "grabber-optimize-bus", () -> grabber_motor_.optimizeBusUtilization(), 5);
    
        coral_ =  new DigitalInterrupt(GrabberConstants.Grabber.CoralSensor.kChannel);
        coral_.enable() ;

        algae_ = new DigitalInterrupt(GrabberConstants.Grabber.AlgaeSensor.kChannel);
        algae_.enable();
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {

        StatusCode grabberStatus = BaseStatusSignal.refreshAll(
            grabber_curr_sig_,
            grabber_vel_sig_,
            grabber_pos_sig_,
            grabber_vol_sig_
        );
        
        inputs.grabberReady = grabberErrorDebounce_.calculate(grabberStatus.isOK());

        inputs.grabberCurrent = grabber_curr_sig_.getValue();
        inputs.grabberVelocity = grabber_vel_sig_.getValue();
        inputs.grabberPosition = grabber_pos_sig_.getValue();
        inputs.grabberVoltage = grabber_vol_sig_.getValue();

        inputs.coralSensor = coral_.value();
        inputs.coralRisingEdge = coral_.risingEdge();
        inputs.coralFallingEdge = coral_.fallingEdge();

        inputs.algaeSensor = algae_.value();
        inputs.algaeRisingEdge = algae_.risingEdge();
        inputs.algaeFallingEdge = algae_.fallingEdge();
    }

    public void logArmMotor(SysIdRoutineLog log) {
        log.motor("grabber")
            .voltage(Units.Volts.of(grabber_voltage_))
            .angularPosition(Revolutions.of(grabber_pos_sig_.refresh().getValueAsDouble()))
            .angularVelocity(RevolutionsPerSecond.of(grabber_vel_sig_.refresh().getValueAsDouble()));
    }

    public void setGrabberMotorVoltage(double vol) {
        grabber_voltage_ = vol;
        grabber_motor_.setVoltage(grabber_voltage_);
    }

    public void setGrabberTargetVelocity(AngularVelocity vel) {
        AngularVelocity cvel = vel.div(GrabberConstants.Grabber.kGearRatio) ;
        grabber_motor_.setControl(new MotionMagicVelocityVoltage(cvel)) ;
    }

    public void setGrabberTargetPosition(Angle pos) {
        Angle cpos = pos.div(GrabberConstants.Grabber.kGearRatio) ;
        grabber_motor_.setControl(new MotionMagicVoltage(cpos)) ;
    }
}
