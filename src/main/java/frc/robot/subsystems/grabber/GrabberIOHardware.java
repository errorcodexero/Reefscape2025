package frc.robot.subsystems.grabber;

import org.xerosw.util.TalonFXFactory;

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
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.util.DigitalInterrupt;

public class GrabberIOHardware implements GrabberIO {
    private DigitalInterrupt coral_front_;
    private DigitalInterrupt coral_back_;
    private DigitalInterrupt coral_funnel_;
    private DigitalInterrupt algae_upper_;
    private DigitalInterrupt algae_lower_;

    private TalonFX grabber_motor_;

    private double grabber_voltage_;

    private StatusSignal<Angle> grabber_pos_sig_;
    private StatusSignal<AngularVelocity> grabber_vel_sig_;
    private StatusSignal<Current> grabber_curr_sig_;
    private StatusSignal<Voltage> grabber_vol_sig_;

    private final Debouncer grabberErrorDebounce_ = new Debouncer(0.5);

    public GrabberIOHardware() throws Exception {

        grabber_motor_ = TalonFXFactory.createTalonFX(GrabberConstants.Grabber.kMotorCANID, null, GrabberConstants.Grabber.kInverted);

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
            null,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                1.0,
                grabber_curr_sig_,
                grabber_pos_sig_,
                grabber_vel_sig_,
                grabber_vol_sig_
            )
        );

        TalonFXFactory.checkError(GrabberConstants.Grabber.kMotorCANID, "grabber-optimize-bus", () -> grabber_motor_.optimizeBusUtilization(), 5);
    
        coral_front_ =  new DigitalInterrupt(GrabberConstants.Grabber.CoralFrontSensor.kChannel);
        coral_back_ = new DigitalInterrupt(GrabberConstants.Grabber.CoralBackSensor.kChannel);
        coral_funnel_ = new DigitalInterrupt(GrabberConstants.Grabber.CoralFunnelSensor.kChannel);
        algae_upper_ = new DigitalInterrupt(GrabberConstants.Grabber.AlgaeUpperSensor.kChannel);
        algae_lower_ = new DigitalInterrupt(GrabberConstants.Grabber.AlgaeLowerSensor.kChannel);
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

        inputs.coralFrontSensor = coral_front_.getSensor();
        inputs.coralFrontRisingEdge = coral_front_.getRising();
        inputs.coralFrontFallingEdge = coral_front_.getFalling();
        
        inputs.coralBackSensor = coral_back_.getSensor();
        inputs.coralBackRisingEdge = coral_back_.getRising();
        inputs.coralBackFallingEdge = coral_back_.getFalling();

        inputs.coralFunnelSensor = coral_funnel_.getSensor();
        inputs.coralFunnelRisingEdge = coral_funnel_.getRising();
        inputs.coralFunnelFallingEdge = coral_funnel_.getFalling();

        inputs.algaeUpperSensor = algae_upper_.getSensor();
        inputs.algaeUpperRisingEdge = algae_upper_.getRising();
        inputs.algaeUpperFallingEdge = algae_upper_.getFalling();

        inputs.algaeLowerSensor = algae_lower_.getSensor();
        inputs.algaeLowerRisingEdge = algae_lower_.getRising();
        inputs.algaeLowerFallingEdge = algae_lower_.getFalling();

        coral_front_.setRising(false);
        coral_back_.setFalling(false);

        coral_back_.setRising(false);
        coral_back_.setFalling(false);

        coral_funnel_.setRising(false);
        coral_funnel_.setFalling(false);

        algae_upper_.setRising(false);
        algae_upper_.setFalling(false);

        algae_lower_.setRising(false);
        algae_lower_.setFalling(false);
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

    
}
