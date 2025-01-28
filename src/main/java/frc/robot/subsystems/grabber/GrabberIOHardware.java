package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.subsystems.grabber.GrabberIO.GrabberIOInputs;

public class GrabberIOHardware {
    private final static double kPositionScale = 100000.0 ;

    // grabber related members
    private TalonFX grabber_motor_ ;
    private Voltage grabber_volts_ ;
    private StatusSignal<Angle> grabber_position_ ;
    private StatusSignal<AngularVelocity> grabber_velocity_ ;
    private StatusSignal<Current> grabber_current_ ;
    private StatusSignal<Voltage> grabber_voltage_ ;

    private DigitalInput sensor_input_ ;
    private AsynchronousInterrupt sensor_interrupt_ ;
    private AtomicBoolean rising_seen_ ;
    private AtomicBoolean falling_seen_ ;
    private AtomicInteger grabber_pos_at_interrupt_ ;

    public GrabberIOHardware() {
        createGrabber() ;

        TalonFXFactory.checkError(-1, "manipulator-update-frequencies-10", () -> BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(10.0),
                        grabber_position_,
                        grabber_velocity_, 
                        grabber_current_,
                        grabber_voltage_)) ;

        grabber_motor_.optimizeBusUtilization() ;                        
                                
        initCoralSensor();
    }
    
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.grabberPosition = grabber_position_.refresh().getValue().times(GrabberConstants.Grabber.kGearRatio) ;
        inputs.grabberVelocity = grabber_velocity_.refresh().getValue().times(GrabberConstants.Grabber.kGearRatio) ;
        inputs.grabberCurrent = grabber_current_.refresh().getValue() ;
        inputs.grabberVoltage = grabber_voltage_.refresh().getValue() ;

        inputs.grabberSensor = sensor_input_.get() ;
        inputs.risingEdge = rising_seen_.get() ;
        inputs.fallingEdge = falling_seen_.get() ;
        inputs.posOnEdge = Revolutions.of(grabber_pos_at_interrupt_.get() / kPositionScale ) ;
    }

    public void setGrabberVelocity(AngularVelocity target) {
        ControlRequest ctrl = new MotionMagicVelocityVoltage(target.div(GrabberConstants.Grabber.kGearRatio)) ;
        grabber_motor_.setControl(ctrl) ;
    }

    public void setGrabberMotorVoltage(double volts) {
        grabber_volts_ = Volts.of(volts) ;
        ControlRequest ctrl = new VoltageOut(grabber_volts_) ;
        grabber_motor_.setControl(ctrl) ;
    }

    public void logGrabberMotor(SysIdRoutineLog log) {
        Angle pos = grabber_position_.refresh().getValue().times(GrabberConstants.Grabber.kGearRatio) ;
        AngularVelocity vel = grabber_velocity_.refresh().getValue().times(GrabberConstants.Grabber.kGearRatio) ;

        log.motor("roller")
            .voltage(grabber_volts_)
            .angularPosition(pos)
            .angularVelocity(vel) ;        
    }

    private void sensorInterruptHandler(boolean rising, boolean falling) {
        if (rising) {
            rising_seen_.set(true) ;
            grabber_pos_at_interrupt_.set((int)(grabber_position_.refresh().getValueAsDouble() * kPositionScale)) ;
        }

        if (falling) {
            falling_seen_.set(true) ;
        }
    }    

    private void initCoralSensor() {
        sensor_input_ = new DigitalInput(GrabberConstants.Sensor.kInput) ;
        sensor_interrupt_ = new AsynchronousInterrupt(sensor_input_, (rising, falling) -> { sensorInterruptHandler(rising, falling); }) ;
        sensor_interrupt_.setInterruptEdges(true, true);
        sensor_interrupt_.enable();

        rising_seen_ = new AtomicBoolean() ;
        falling_seen_ = new AtomicBoolean() ;
        grabber_pos_at_interrupt_ = new AtomicInteger() ;
    }

    private void createGrabber() {
        TalonFXFactory f = TalonFXFactory.getFactory() ;
        grabber_motor_ = f.createTalonFX(GrabberConstants.Grabber.kMotorCANID, GrabberConstants.Grabber.kInverted) ;

        grabber_position_ = grabber_motor_.getPosition() ;
        grabber_velocity_ = grabber_motor_.getVelocity() ;
        grabber_current_ = grabber_motor_.getSupplyCurrent() ;
        grabber_voltage_ = grabber_motor_.getMotorVoltage() ;        

        // ELEVATOR CONFIGS:
        Slot0Configs grabber_pids = new Slot0Configs();
        grabber_pids.kP = GrabberConstants.Grabber.PID.kP;
        grabber_pids.kI = GrabberConstants.Grabber.PID.kI;
        grabber_pids.kD = GrabberConstants.Grabber.PID.kD;
        grabber_pids.kV = GrabberConstants.Grabber.PID.kV;
        grabber_pids.kA = GrabberConstants.Grabber.PID.kA;
        grabber_pids.kG = GrabberConstants.Grabber.PID.kG;
        grabber_pids.kS = GrabberConstants.Grabber.PID.kS;
        TalonFXFactory.checkError(GrabberConstants.Grabber.kMotorCANID, "apply", () -> grabber_motor_.getConfigurator().apply(grabber_pids)) ;

        MotionMagicConfigs rollerMotionMagicConfigs = new MotionMagicConfigs();
        rollerMotionMagicConfigs.MotionMagicCruiseVelocity = GrabberConstants.Grabber.MotionMagic.kMaxVelocity;
        rollerMotionMagicConfigs.MotionMagicAcceleration = GrabberConstants.Grabber.MotionMagic.kMaxAcceleration;
        rollerMotionMagicConfigs.MotionMagicJerk = GrabberConstants.Grabber.MotionMagic.kJerk;
        TalonFXFactory.checkError(GrabberConstants.Grabber.kMotorCANID, "apply", () -> grabber_motor_.getConfigurator().apply(rollerMotionMagicConfigs)) ;
    }    
}
