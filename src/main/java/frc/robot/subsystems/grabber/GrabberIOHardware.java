package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.xerosw.util.DigitalInterrupt;
import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Robot;
import frc.robot.subsystems.manipulator.ManipulatorConstants;

public class GrabberIOHardware implements GrabberIO {
    private final static double kPositionScale = 100000.0 ;

    // grabber related members
    private TalonFX grabber_motor_ ;
    private Voltage grabber_volts_ ;
    private StatusSignal<Angle> grabber_position_ ;
    private StatusSignal<AngularVelocity> grabber_velocity_ ;
    private StatusSignal<Current> grabber_current_ ;
    private StatusSignal<Voltage> grabber_voltage_ ;

    private DCMotorSim grabber_sim_ ;

    private DigitalInterrupt coral_sensor_low_ ;
    private DigitalInterrupt coral_sensor_high_ ;
    private DigitalInterrupt coral_sensor_funnel_ ;
    private DigitalInterrupt algae_sensor_low_ ;
    private DigitalInterrupt algae_sensor_high_ ;

    public GrabberIOHardware() {
        createGrabber() ;

        TalonFXFactory.checkError(-1, "manipulator-update-frequencies-10", () -> BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(10.0),
                        grabber_position_,
                        grabber_velocity_, 
                        grabber_current_,
                        grabber_voltage_)) ;

        grabber_motor_.optimizeBusUtilization() ;                        
                                
        initSensors();
    }
    
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.grabberPosition = grabber_position_.refresh().getValue().div(GrabberConstants.Grabber.kGearRatio) ;
        inputs.grabberVelocity = grabber_velocity_.refresh().getValue().div(GrabberConstants.Grabber.kGearRatio) ;
        inputs.grabberRawVelocity = grabber_velocity_.refresh().getValue() ;
        inputs.grabberCurrent = grabber_current_.refresh().getValue() ;
        inputs.grabberVoltage = grabber_voltage_.refresh().getValue() ;

        inputs.algaeLow = algae_sensor_low_.getInput().get() ;
        inputs.algaeLowRisingEdge = algae_sensor_low_.risingEdge();
        inputs.algaeLowFallingEdge = algae_sensor_low_.fallingEdge();

        inputs.algaeHigh = algae_sensor_high_.getInput().get() ;
        inputs.algaeHighRisingEdge = algae_sensor_high_.risingEdge();
        inputs.algaeHighFallingEdge = algae_sensor_high_.fallingEdge();

        inputs.coralSensorLow = coral_sensor_low_.getInput().get() ;
        inputs.coralSensorLowRisingEdge = coral_sensor_low_.risingEdge();
        inputs.coralSensorLowFallingEdge = coral_sensor_low_.fallingEdge();
        inputs.grabberPositionCoralSensorLowEdge = Rotations.of(coral_sensor_low_.getCount() / kPositionScale) ;

        inputs.coralHigh = coral_sensor_high_.getInput().get() ;
        inputs.coralHighRisingEdge = coral_sensor_high_.risingEdge();
        inputs.coralHighFallingEdge = coral_sensor_high_.fallingEdge();
        inputs.grabberPositionCoralSensorHighEdge = Rotations.of(coral_sensor_high_.getCount() / kPositionScale) ;

        inputs.coralFunnel = coral_sensor_funnel_.getInput().get() ;
        inputs.coralFunnelRisingEdge = coral_sensor_funnel_.risingEdge();
        inputs.coralFunnelFallingEdge = coral_sensor_funnel_.fallingEdge();

        simulateGrabber();
    }

    public void setGrabberTargetVelocity(AngularVelocity target) {
        ControlRequest ctrl = new MotionMagicVelocityVoltage(target.times(GrabberConstants.Grabber.kGearRatio)).withSlot(0).withEnableFOC(true) ;
        grabber_motor_.setControl(ctrl) ;
    }

    public void setGrabberTargetPosition(Angle target) {
        ControlRequest ctrl = new MotionMagicVoltage(target.times(GrabberConstants.Grabber.kGearRatio)).withSlot(1).withEnableFOC(true) ;
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

    private void initSensors() {
        coral_sensor_low_ = new DigitalInterrupt(GrabberConstants.Sensor.kCoralLow, true, true) ;
        coral_sensor_low_.setCountSupplier(()->(int)(grabber_position_.refresh().getValue().in(Revolutions) * kPositionScale)) ;
        coral_sensor_low_.enable() ;

        coral_sensor_high_ = new DigitalInterrupt(GrabberConstants.Sensor.kCoralHigh, true, true) ;
        coral_sensor_high_.setCountSupplier(()->(int)(grabber_position_.refresh().getValue().in(Revolutions) * kPositionScale)) ;
        coral_sensor_high_.enable() ;

        coral_sensor_funnel_ = new DigitalInterrupt(GrabberConstants.Sensor.kCoralFunnel, true, true) ;
        coral_sensor_funnel_.enable() ;

        algae_sensor_low_ = new DigitalInterrupt(GrabberConstants.Sensor.kAlgaeLow, true, true) ;
        algae_sensor_low_.enable() ;

        algae_sensor_high_ = new DigitalInterrupt(GrabberConstants.Sensor.kAlgaeHigh, true, true) ;
        algae_sensor_high_.enable() ;
    }

    private void createGrabber() {
        TalonFXFactory f = TalonFXFactory.getFactory() ;
        grabber_motor_ = f.createTalonFX(GrabberConstants.Grabber.kMotorCANID, 
                                         GrabberConstants.Grabber.kInverted,
                                         GrabberConstants.Grabber.kCurrentLimit) ;

        grabber_position_ = grabber_motor_.getPosition() ;
        grabber_velocity_ = grabber_motor_.getVelocity() ;
        grabber_current_ = grabber_motor_.getSupplyCurrent() ;
        grabber_voltage_ = grabber_motor_.getMotorVoltage() ;        

        // ELEVATOR CONFIGS:
        Slot0Configs grabber_velocity_pids = new Slot0Configs();
        grabber_velocity_pids.kP = GrabberConstants.Grabber.Velocity.PID.kP;
        grabber_velocity_pids.kI = GrabberConstants.Grabber.Velocity.PID.kI;
        grabber_velocity_pids.kD = GrabberConstants.Grabber.Velocity.PID.kD;
        grabber_velocity_pids.kV = GrabberConstants.Grabber.Velocity.PID.kV;
        grabber_velocity_pids.kA = GrabberConstants.Grabber.Velocity.PID.kA;
        grabber_velocity_pids.kG = GrabberConstants.Grabber.Velocity.PID.kG;
        grabber_velocity_pids.kS = GrabberConstants.Grabber.Velocity.PID.kS;
        TalonFXFactory.checkError(GrabberConstants.Grabber.kMotorCANID, "apply", () -> grabber_motor_.getConfigurator().apply(grabber_velocity_pids)) ;

        Slot1Configs grabber_position_pids = new Slot1Configs();
        grabber_position_pids.kP = GrabberConstants.Grabber.Position.PID.kP;
        grabber_position_pids.kI = GrabberConstants.Grabber.Position.PID.kI;
        grabber_position_pids.kD = GrabberConstants.Grabber.Position.PID.kD;
        grabber_position_pids.kV = GrabberConstants.Grabber.Position.PID.kV;
        grabber_position_pids.kA = GrabberConstants.Grabber.Position.PID.kA;
        grabber_position_pids.kG = GrabberConstants.Grabber.Position.PID.kG;
        grabber_position_pids.kS = GrabberConstants.Grabber.Position.PID.kS;
        TalonFXFactory.checkError(GrabberConstants.Grabber.kMotorCANID, "apply", () -> grabber_motor_.getConfigurator().apply(grabber_position_pids)) ;        

        MotionMagicConfigs rollerMotionMagicConfigs = new MotionMagicConfigs();
        rollerMotionMagicConfigs.MotionMagicCruiseVelocity = GrabberConstants.Grabber.MotionMagic.kMaxVelocity;
        rollerMotionMagicConfigs.MotionMagicAcceleration = GrabberConstants.Grabber.MotionMagic.kMaxAcceleration;
        rollerMotionMagicConfigs.MotionMagicJerk = GrabberConstants.Grabber.MotionMagic.kJerk;
        TalonFXFactory.checkError(GrabberConstants.Grabber.kMotorCANID, "apply", () -> grabber_motor_.getConfigurator().apply(rollerMotionMagicConfigs)) ;

        if (Robot.isSimulation()) {
            LinearSystem<N2, N1, N2> sys = LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 
                                                                              GrabberConstants.Grabber.kMOI.in(KilogramSquareMeters), 
                                                                              GrabberConstants.Grabber.kGearRatio) ;
            grabber_sim_ = new DCMotorSim(sys, DCMotor.getKrakenX60Foc(1)) ;
        }
    }    

    private void simulateGrabber() {
        TalonFXSimState st = grabber_motor_.getSimState() ;
        st.setSupplyVoltage(RobotController.getBatteryVoltage()) ;

        Voltage mv = st.getMotorVoltageMeasure() ;
        grabber_sim_.setInputVoltage(mv.in(Volts)) ;
        grabber_sim_.update(0.02) ;
        st.setRawRotorPosition(grabber_sim_.getAngularPosition().times(ManipulatorConstants.Arm.kGearRatio)) ;
        st.setRotorVelocity(grabber_sim_.getAngularVelocity().times(ManipulatorConstants.Arm.kGearRatio)) ;        
    }
}
