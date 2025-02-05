package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

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

    private DigitalInput cage_sensor_1_ ;
    private DigitalInput cage_sensor_2_ ;
    private DigitalInput cage_sensor_3_ ;
    private DigitalInput door_sensor_1_ ;
    private DigitalInput door_sensor_2_ ;


    public ClimberIOHardware() {
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

    private void createClimber() {
        TalonFXFactory f = TalonFXFactory.getFactory() ;
        motor_ = f.createTalonFX(ClimberConstants.ClimberArm.kMotorCANID, 
                                         ClimberConstants.ClimberArm.kInverted,
                                         ClimberConstants.ClimberArm.kCurrentLimit) ;

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
    } 

    private void initSensors() {
        cage_sensor_1_ = new DigitalInput(ClimberConstants.Sensors.kCageSensor1) ;
        cage_sensor_2_ = new DigitalInput(ClimberConstants.Sensors.kCageSensor2) ;
        cage_sensor_3_ = new DigitalInput(ClimberConstants.Sensors.kCageSensor3) ;
        door_sensor_1_ = new DigitalInput(ClimberConstants.Sensors.kDoorSensor1) ;
        door_sensor_2_ = new DigitalInput(ClimberConstants.Sensors.kDoorSensor2) ;
    }
}
