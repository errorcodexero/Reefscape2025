package frc.robot.subsystems.climber;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Robot;
import frc.robot.subsystems.manipulator.ManipulatorConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;
import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class ClimberIOHardware implements ClimberIO {
    private TalonFX motor_;
    private Voltage volts_ ;

    private StatusSignal<Angle> position_ ;
    private StatusSignal<AngularVelocity> velocity_ ;
    private StatusSignal<Current> current_ ;
    private StatusSignal<Voltage> voltage_ ;

    private DCMotorSim climber_sim_ ;

    private DigitalInput cage_sensor_1_ ;
    private DigitalInput cage_sensor_2_ ;
    private DigitalInput cage_sensor_3_ ;
    private DigitalInput door_sensor_1_ ;
    private DigitalInput door_sensor_2_ ;


    public ClimberIOHardware() throws Exception {
        createClimber() ;
        initSensors() ;
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs_) {
        inputs_.climberPosition = position_.refresh().getValue() ;
        inputs_.climberVelocity = velocity_.refresh().getValue() ;
        inputs_.climberCurrent = current_.refresh().getValue() ;
        inputs_.climberVoltage = voltage_.refresh().getValue() ;

        inputs_.cage_sensor_1_ = cage_sensor_1_.get() ;
        inputs_.cage_sensor_2_ = cage_sensor_2_.get() ;
        inputs_.cage_sensor_3_ = cage_sensor_3_.get() ;
        inputs_.door_sensor_1_ = door_sensor_1_.get() ;
        inputs_.door_sensor_2_ = door_sensor_2_.get() ;

        simulateClimber() ;
    }

    public void setClimberPosition(Angle target) {
        ControlRequest ctrl = new MotionMagicVoltage(target.div(ClimberConstants.ClimberArm.kGearRatio)).withEnableFOC(true) ;
        motor_.setControl(ctrl) ;
    }

    public void setClimberMotorVoltage(double volts) {
        volts_ = Volts.of(volts) ;
        ControlRequest ctrl = new VoltageOut(volts_) ;
        motor_.setControl(ctrl) ;
    }

    public void logClimberMotor(SysIdRoutineLog log) {
        Angle pos = position_.refresh().getValue().times(ClimberConstants.ClimberArm.kGearRatio) ;
        AngularVelocity vel = velocity_.refresh().getValue().times(ClimberConstants.ClimberArm.kGearRatio) ;

        log.motor("roller")
            .voltage(volts_)
            .angularPosition(pos)
            .angularVelocity(vel) ;        
    }

    private void createClimber() throws Exception {
        motor_ = TalonFXFactory.createTalonFX(ClimberConstants.ClimberArm.kMotorCANID,
                                         ClimberConstants.ClimberArm.kInverted,
                                         ClimberConstants.ClimberArm.kCurrentLimit,
                                         ClimberConstants.ClimberArm.kCurrentLimitTime) ;

        position_ = motor_.getPosition() ;
        velocity_ = motor_.getVelocity() ;
        current_ = motor_.getSupplyCurrent() ;
        voltage_ = motor_.getMotorVoltage() ;        

        // ELEVATOR CONFIGS:
        Slot0Configs pids = new Slot0Configs();
        pids.kP = ClimberConstants.ClimberArm.PID.kP;
        pids.kI = ClimberConstants.ClimberArm.PID.kI;
        pids.kD = ClimberConstants.ClimberArm.PID.kD;
        pids.kV = ClimberConstants.ClimberArm.PID.kV;
        pids.kA = ClimberConstants.ClimberArm.PID.kA;
        pids.kG = ClimberConstants.ClimberArm.PID.kG;
        pids.kS = ClimberConstants.ClimberArm.PID.kS;
        TalonFXFactory.checkError(ClimberConstants.ClimberArm.kMotorCANID, "apply", () -> motor_.getConfigurator().apply(pids)) ;

        MotionMagicConfigs rollerMotionMagicConfigs = new MotionMagicConfigs();
        rollerMotionMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.ClimberArm.MotionMagic.kMaxVelocity;
        rollerMotionMagicConfigs.MotionMagicAcceleration = ClimberConstants.ClimberArm.MotionMagic.kMaxAcceleration;
        rollerMotionMagicConfigs.MotionMagicJerk = ClimberConstants.ClimberArm.MotionMagic.kJerk;
        TalonFXFactory.checkError(ClimberConstants.ClimberArm.kMotorCANID, "apply", () -> motor_.getConfigurator().apply(rollerMotionMagicConfigs)) ;

        if (Robot.isSimulation()) {
            LinearSystem<N2, N1, N2> sys = LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 
                                                                              ClimberConstants.ClimberArm.kMOI.in(KilogramSquareMeters), 
                                                                              ClimberConstants.ClimberArm.kGearRatio) ;
            climber_sim_ = new DCMotorSim(sys, DCMotor.getKrakenX60Foc(1)) ;
        }
    } 

    private void initSensors() {
        cage_sensor_1_ = new DigitalInput(ClimberConstants.Sensors.kCageSensor1) ;
        cage_sensor_2_ = new DigitalInput(ClimberConstants.Sensors.kCageSensor2) ;
        cage_sensor_3_ = new DigitalInput(ClimberConstants.Sensors.kCageSensor3) ;
        door_sensor_1_ = new DigitalInput(ClimberConstants.Sensors.kDoorSensor1) ;
        door_sensor_2_ = new DigitalInput(ClimberConstants.Sensors.kDoorSensor2) ;
    }

    private void simulateClimber() {
        TalonFXSimState st = motor_.getSimState() ;
        st.setSupplyVoltage(RobotController.getBatteryVoltage()) ;

        Voltage mv = st.getMotorVoltageMeasure() ;
        climber_sim_.setInputVoltage(mv.in(Volts)) ;
        climber_sim_.update(0.02) ;
        st.setRawRotorPosition(climber_sim_.getAngularPosition().times(ManipulatorConstants.Arm.kGearRatio)) ;
        st.setRotorVelocity(climber_sim_.getAngularVelocity().times(ManipulatorConstants.Arm.kGearRatio)) ;        
    }     
}
