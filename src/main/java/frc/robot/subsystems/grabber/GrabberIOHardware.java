package frc.robot.subsystems.grabber;

import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import org.xerosw.util.TalonFXFactory;

public class GrabberIOHardware implements GrabberIO {
    private TalonFX grabber_motor_;

    private DigitalInput coral_sensor_;
    private DigitalInput algae_sensor_;

    private AsynchronousInterrupt coral_interrupt_;
    private AsynchronousInterrupt algae_interrupt_;

    private AtomicBoolean coral_rising_seen_;
    private AtomicBoolean coral_falling_seen_;
    private AtomicBoolean algae_rising_seen_;
    private AtomicBoolean algae_falling_seen_;

    private double grabber_voltage_;

    private StatusSignal<Angle> grabber_pos_sig;
    private StatusSignal<AngularVelocity> grabber_vel_sig;
    private StatusSignal<Current> grabber_curr_sig;
    private StatusSignal<Voltage> grabber_vol_sig;

    private final Debouncer grabberReadyDebounce_ = new Debouncer(0.5);
    private boolean grabberInitialized_ = true;

    public GrabberIOHardware() {
        try {
            grabber_motor_ = TalonFXFactory.createTalonFX(GrabberConstants.Grabber.kMotorCANID, null, GrabberConstants.Grabber.kInverted);
        } catch (Exception e) {
            grabberInitialized_ = false; // If fail, set init to false.
        }
            
        Slot0Configs grabber_pids_ = new Slot0Configs();
        grabber_pids_.kP = GrabberConstants.Grabber.PID.kP;
        grabber_pids_.kI = GrabberConstants.Grabber.PID.kI;
        grabber_pids_.kD = GrabberConstants.Grabber.PID.kD;
        grabber_pids_.kV = GrabberConstants.Grabber.PID.kV;
        grabber_pids_.kA = GrabberConstants.Grabber.PID.kA;
        grabber_pids_.kG = GrabberConstants.Grabber.PID.kG;
        grabber_pids_.kS = GrabberConstants.Grabber.PID.kS;
        grabber_motor_.getConfigurator().apply(grabber_pids_);

        MotionMagicConfigs grabberMotionMagicConfigs = new MotionMagicConfigs();
        grabberMotionMagicConfigs.MotionMagicCruiseVelocity = GrabberConstants.Grabber.MotionMagic.kMaxVelocity;
        grabberMotionMagicConfigs.MotionMagicAcceleration = GrabberConstants.Grabber.MotionMagic.kMaxAcceleration;
        grabberMotionMagicConfigs.MotionMagicJerk = GrabberConstants.Grabber.MotionMagic.kJerk;
        grabber_motor_.getConfigurator().apply(grabberMotionMagicConfigs);

        try {
            TalonFXFactory.checkError(
                GrabberConstants.Grabber.kMotorCANID,
                null,
                () -> BaseStatusSignal.setUpdateFrequencyForAll(1.0,
                grabber_curr_sig,
                grabber_pos_sig,
                grabber_vel_sig,
                grabber_vol_sig)
            );
        } catch (Exception e) {
            grabberInitialized_ = false; // If fail, set init to false.
        }

        coral_sensor_ = new DigitalInput(GrabberConstants.Grabber.CoralSensor.kChannel);
        algae_sensor_ = new DigitalInput(GrabberConstants.Grabber.AlgaeSensor.kChannel);

        coral_rising_seen_ = new AtomicBoolean();
        coral_falling_seen_ = new AtomicBoolean();
        algae_rising_seen_ = new AtomicBoolean();
        algae_falling_seen_ = new AtomicBoolean();

        coral_interrupt_ = new AsynchronousInterrupt(coral_sensor_, (coral_rising_, coral_falling_) -> {coralInterruptHandler(coral_rising_, coral_falling_);});
        algae_interrupt_ = new AsynchronousInterrupt(algae_sensor_, (algae_rising_, algae_falling_) -> {coralInterruptHandler(algae_rising_, algae_falling_);});

        coral_interrupt_.setInterruptEdges(true, true);
        algae_interrupt_.setInterruptEdges(true, true);

        coral_interrupt_.enable();
        algae_interrupt_.enable();
    }

     @Override
    public void updateInputs(GrabberIOInputs inputs) {

        // Refresh all signals.
        StatusCode grabberStatus = BaseStatusSignal.refreshAll(grabber_curr_sig, grabber_vel_sig, grabber_pos_sig, grabber_vol_sig);

        inputs.grabberReady = grabberReadyDebounce_.calculate(grabberStatus.isOK()) && grabberInitialized_;

        inputs.grabberCurrent = grabber_curr_sig.getValue();
        inputs.grabberVelocity = grabber_vel_sig.getValue();
        inputs.grabberPosition = grabber_pos_sig.getValue();
        inputs.grabberVoltage = grabber_vol_sig.getValue();

        inputs.coralRisingEdge = coral_rising_seen_.get();
        inputs.coralRisingEdge = coral_falling_seen_.get();

        inputs.algaeRisingEdge = algae_rising_seen_.get();
        inputs.algaeRisingEdge = algae_falling_seen_.get();

        inputs.coralSensor = coral_sensor_.get();
        inputs.algaeSensor = algae_sensor_.get();

        coral_rising_seen_.set(false);
        coral_falling_seen_.set(false);
        algae_rising_seen_.set(false);
        algae_falling_seen_.set(false);
    }

    public void logArmMotor(SysIdRoutineLog log) {
        // code goes here, look at documentation 
    }

    public void setGrabberMotorVoltage(double vol) {
        grabber_voltage_ = vol;
        grabber_motor_.setVoltage(grabber_voltage_);
    }

    public void setGrabberTargetVelocity(double vel) {
        double cvel = vel / GrabberConstants.Grabber.kGearRatio;
        grabber_motor_.setControl(new MotionMagicVelocityVoltage(cvel));
    }

    public void coralInterruptHandler(boolean rising, boolean falling) {
        if (rising) {
            if (RobotBase.isReal()){
                coral_rising_seen_.set(true);
            }
            else {
                coral_falling_seen_.set(true);
            }
        }

        if (falling) {
            if (RobotBase.isReal()) {
                coral_falling_seen_.set(true);
            }
            else {
                coral_rising_seen_.set(true);
            }
        }
    }

    public void algaeInterruptHandler(boolean rising, boolean falling) {
        if (rising) {
            if (RobotBase.isReal()){
                algae_rising_seen_.set(true);
            }
            else {
                algae_falling_seen_.set(true);
            }
        }

        if (falling) {
            if (RobotBase.isReal()) {
                algae_falling_seen_.set(true);
            }
            else {
                algae_rising_seen_.set(true);
            }
        }
    }
}
