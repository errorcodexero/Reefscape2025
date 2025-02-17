package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;

import org.xerosw.util.EncoderMapper;
import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Robot;

public class ManipulatorIOHardware implements ManipulatorIO {
    private TalonFX arm_motor_; 
    private TalonFX elevator_motor_; 
    private TalonFX elevator_motor_2_;
    private DutyCycleEncoder encoder_; 
    private EncoderMapper mapper_; 
    private DigitalInput hall_effect_sensor_ ;

    private DCMotorSim arm_sim_ ;
    private DCMotorSim elevator_sim_ ;
    private DutyCycleEncoderSim arm_encoder_sim_ ;
    private boolean encoder_motor_synced_ ;

    private StatusSignal<Angle> arm_pos_sig_; 
    private StatusSignal<AngularVelocity> arm_vel_sig_; 
    private StatusSignal<Voltage> arm_vol_sig_; 
    private StatusSignal<Current> arm_current_sig_;

    private StatusSignal<Angle> elevator_pos_sig_; 
    private StatusSignal<AngularVelocity> elevator_vel_sig_; 
    private StatusSignal<Voltage> elevator_1_vol_sig_; 
    private StatusSignal<Current> elevator_1_current_sig_;

    private StatusSignal<Voltage> elevator_2_vol_sig_; 
    private StatusSignal<Current> elevator_2_current_sig_;

    private Voltage arm_voltage_; 
    private Voltage elevator_voltage_; 

    private final Debouncer armErrorDebounce_ = new Debouncer(0.5);
    private final Debouncer elevator1ErrorDebounce_ = new Debouncer(0.5);
    private final Debouncer elevator2ErrorDebounce_ = new Debouncer(0.5);

    public ManipulatorIOHardware() throws Exception {
        createArm() ;
        createElevator() ;   

        // setting signal update frequency: 
        TalonFXFactory.checkError(-1, "set-manipulator-frequency", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                arm_pos_sig_,
                arm_vel_sig_,
                arm_vol_sig_,
                arm_current_sig_,
                elevator_pos_sig_,
                elevator_vel_sig_,
                elevator_1_vol_sig_,
                elevator_1_current_sig_,
                elevator_2_vol_sig_,
                elevator_2_current_sig_
            )
        );

    }

    private void createElevator() throws Exception {
        elevator_motor_ = TalonFXFactory.createTalonFX(
            ManipulatorConstants.Elevator.kMotorFrontCANID,
            ManipulatorConstants.Elevator.kCANBusName,
            ManipulatorConstants.Elevator.kInverted,
            ManipulatorConstants.Elevator.kCurrentLimit,
            ManipulatorConstants.Elevator.kCurrentLimitTime
        );
        elevator_motor_.setPosition(Degrees.of(0.0)) ;

        elevator_motor_2_ = TalonFXFactory.createTalonFX(
            ManipulatorConstants.Elevator.kMotorBackCANID,
            ManipulatorConstants.Elevator.kCANBusName,
            ManipulatorConstants.Elevator.kInverted,
            ManipulatorConstants.Elevator.kCurrentLimit,
            ManipulatorConstants.Elevator.kCurrentLimitTime
        );
        elevator_motor_2_.setControl(new Follower(ManipulatorConstants.Elevator.kMotorFrontCANID, false));

        // ELEVATOR CONFIGS:
        Slot0Configs elevator_pids = new Slot0Configs();
        elevator_pids.kP = ManipulatorConstants.Elevator.PID.kP;
        elevator_pids.kI = ManipulatorConstants.Elevator.PID.kI;
        elevator_pids.kD = ManipulatorConstants.Elevator.PID.kD;
        elevator_pids.kV = ManipulatorConstants.Elevator.PID.kV;
        elevator_pids.kA = ManipulatorConstants.Elevator.PID.kA;
        elevator_pids.kG = ManipulatorConstants.Elevator.PID.kG;
        elevator_pids.kS = ManipulatorConstants.Elevator.PID.kS;
        
        MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = ManipulatorConstants.Elevator.MotionMagic.kMaxVelocity.in(RotationsPerSecond) ;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = ManipulatorConstants.Elevator.MotionMagic.kMaxAcceleration.in(RotationsPerSecondPerSecond) ;
        elevatorMotionMagicConfigs.MotionMagicJerk = ManipulatorConstants.Elevator.MotionMagic.kJerk ;

        SoftwareLimitSwitchConfigs elevatorLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        elevatorLimitSwitchConfigs.ForwardSoftLimitEnable = true;
        elevatorLimitSwitchConfigs.ForwardSoftLimitThreshold = ManipulatorConstants.Elevator.kMaxHeight.in(Meters) / ManipulatorConstants.Elevator.kMetersPerRev;
        elevatorLimitSwitchConfigs.ReverseSoftLimitEnable = true;
        elevatorLimitSwitchConfigs.ReverseSoftLimitThreshold = ManipulatorConstants.Elevator.kMinHeight.in(Meters) / ManipulatorConstants.Elevator.kMetersPerRev;

        TalonFXFactory.checkError(ManipulatorConstants.Elevator.kMotorFrontCANID, "set-elevator-PID-values", () -> elevator_motor_.getConfigurator().apply(elevator_pids));
        TalonFXFactory.checkError(ManipulatorConstants.Elevator.kMotorFrontCANID, "set-elevator-MM-values", () -> elevator_motor_.getConfigurator().apply(elevatorMotionMagicConfigs));
        TalonFXFactory.checkError(ManipulatorConstants.Elevator.kMotorFrontCANID, "set-elevator-limit-values", () -> elevator_motor_.getConfigurator().apply(elevatorMotionMagicConfigs)) ;

        elevator_pos_sig_ = elevator_motor_.getPosition();
        elevator_vel_sig_ = elevator_motor_.getVelocity();
        elevator_1_vol_sig_ = elevator_motor_.getMotorVoltage();
        elevator_1_current_sig_ = elevator_motor_.getSupplyCurrent();
        elevator_2_vol_sig_ = elevator_motor_2_.getMotorVoltage();
        elevator_2_current_sig_ = elevator_motor_2_.getSupplyCurrent();

        hall_effect_sensor_ = new DigitalInput(ManipulatorConstants.Elevator.kHallEffectSensorChannel);
        
        if (Robot.isSimulation()) {
            LinearSystem<N2, N1, N2> sys = LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 
                                                     ManipulatorConstants.Elevator.kMOI.in(KilogramSquareMeters), 
                                                     ManipulatorConstants.Elevator.kGearRatio) ;

            elevator_sim_ = new DCMotorSim(sys, DCMotor.getKrakenX60Foc(2)) ;
        }

    }

    private void createArm() throws Exception {

        encoder_motor_synced_ = false ;

        arm_motor_ = TalonFXFactory.createTalonFX(
            ManipulatorConstants.Arm.kMotorCANID,
            ManipulatorConstants.Arm.kCANBusName,
            ManipulatorConstants.Arm.kInverted,
            ManipulatorConstants.Arm.kCurrentLimit,
            ManipulatorConstants.Arm.kCurrentLimitTime
        );


        // ENCODER + MAPPER
        encoder_ = new DutyCycleEncoder(ManipulatorConstants.Arm.ThruBoreEncoder.kEncoderSource); 

        mapper_ = new EncoderMapper(
            ManipulatorConstants.Arm.ThruBoreEncoder.kRobotMax,
            ManipulatorConstants.Arm.ThruBoreEncoder.kRobotMin,
            ManipulatorConstants.Arm.ThruBoreEncoder.kEncoderMax,
            ManipulatorConstants.Arm.ThruBoreEncoder.kEncoderMin
        );

        // ENCODER CONFIGS: 
        mapper_.calibrate(ManipulatorConstants.Arm.ThruBoreEncoder.kRobotCalibrationValue,
        ManipulatorConstants.Arm.ThruBoreEncoder.kEncoderCalibrationValue);        

        // ARM CONFIGS: 
        Slot0Configs arm_pids = new Slot0Configs();
        arm_pids.kP = ManipulatorConstants.Arm.PID.kP; 
        arm_pids.kI = ManipulatorConstants.Arm.PID.kI; 
        arm_pids.kD = ManipulatorConstants.Arm.PID.kD; 
        arm_pids.kV = ManipulatorConstants.Arm.PID.kV; 
        arm_pids.kA = ManipulatorConstants.Arm.PID.kA; 
        arm_pids.kG = ManipulatorConstants.Arm.PID.kG; 
        arm_pids.kS = ManipulatorConstants.Arm.PID.kS; 
        
        MotionMagicConfigs armMotionMagicConfigs = new MotionMagicConfigs(); 
        armMotionMagicConfigs.MotionMagicCruiseVelocity = ManipulatorConstants.Arm.MotionMagic.kMaxVelocity.in(RotationsPerSecond) ;
        armMotionMagicConfigs.MotionMagicAcceleration = ManipulatorConstants.Arm.MotionMagic.kMaxAcceleration.in(RotationsPerSecondPerSecond) ;
        armMotionMagicConfigs.MotionMagicJerk = ManipulatorConstants.Arm.MotionMagic.kJerk;

        SoftwareLimitSwitchConfigs armLimitSwitchConfig = new SoftwareLimitSwitchConfigs();
        armLimitSwitchConfig.ForwardSoftLimitEnable = true;
        armLimitSwitchConfig.ForwardSoftLimitThreshold = ManipulatorConstants.Arm.kMaxArmAngle.times(ManipulatorConstants.Arm.kGearRatio).in(Rotations);
        armLimitSwitchConfig.ReverseSoftLimitEnable = true;
        armLimitSwitchConfig.ReverseSoftLimitThreshold = ManipulatorConstants.Arm.kMinArmAngle.times(ManipulatorConstants.Arm.kGearRatio).in(Rotations);
    

        TalonFXFactory.checkError(ManipulatorConstants.Arm.kMotorCANID, "set-arm-PID-values", () -> arm_motor_.getConfigurator().apply(arm_pids));
        TalonFXFactory.checkError(ManipulatorConstants.Arm.kMotorCANID, "set-arm-MM-values", () -> arm_motor_.getConfigurator().apply(armMotionMagicConfigs));
        TalonFXFactory.checkError(ManipulatorConstants.Arm.kMotorCANID, "set-arm-limits", () -> arm_motor_.getConfigurator().apply(armLimitSwitchConfig)) ;

        arm_pos_sig_ = arm_motor_.getPosition();
        arm_vel_sig_ = arm_motor_.getVelocity();
        arm_vol_sig_ = arm_motor_.getMotorVoltage();
        arm_current_sig_ = arm_motor_.getSupplyCurrent();

        if (Robot.isSimulation()) {
            LinearSystem<N2, N1, N2> sys = LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 
                                                                              ManipulatorConstants.Arm.kMOI.in(KilogramSquareMeters), 
                                                                              ManipulatorConstants.Arm.kGearRatio) ;
            arm_sim_ = new DCMotorSim(sys, DCMotor.getKrakenX60Foc(1)) ;

            arm_encoder_sim_ = new DutyCycleEncoderSim(encoder_) ;
            arm_encoder_sim_.set(mapper_.toEncoder(ManipulatorConstants.Arm.kStartAbsEncoderAngle.in(Degrees))) ;
        }        
    }

    // updates all of the inputs from ManipulatorIO 
    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {

        if (!encoder_motor_synced_ && RobotState.isEnabled()) {
            syncArmPosition() ;
            encoder_motor_synced_ = true ;
        }

        StatusCode armStatus = BaseStatusSignal.refreshAll(
            arm_pos_sig_,
            arm_vel_sig_,
            arm_vol_sig_,
            arm_current_sig_
        );

        StatusCode elevator1Status = BaseStatusSignal.refreshAll(
            elevator_pos_sig_,
            elevator_vel_sig_,
            elevator_1_vol_sig_,
            elevator_1_current_sig_
        );

        StatusCode elevator2Status = BaseStatusSignal.refreshAll(
            elevator_2_vol_sig_,
            elevator_2_current_sig_
        );


        inputs.elevator1Ready = elevator1ErrorDebounce_.calculate(elevator1Status.isOK());
        inputs.elevator2Ready = elevator2ErrorDebounce_.calculate(elevator2Status.isOK());

        // arm inputs:
        inputs.armReady = armErrorDebounce_.calculate(armStatus.isOK());
        inputs.armRawMotorPosition = arm_pos_sig_.getValue() ;
        inputs.armRawMotorVelocity = arm_vel_sig_.getValue() ;
        inputs.armPosition = inputs.armRawMotorPosition.div(ManipulatorConstants.Arm.kGearRatio);
        inputs.armVelocity = inputs.armRawMotorVelocity.div(ManipulatorConstants.Arm.kGearRatio);
        inputs.armVoltage = arm_vol_sig_.getValue();
        inputs.armCurrent = arm_current_sig_.getValue();
        
        // elevator inputs:
        double rev = elevator_pos_sig_.getValue().in(Revolution); 
        inputs.elevatorPosition = Meters.of(rev * ManipulatorConstants.Elevator.kMetersPerRev);
        inputs.elevatorRawMotorPosition = elevator_pos_sig_.getValue() ;
        inputs.elevatorRawMotorVelocity = elevator_vel_sig_.getValue() ;

        double vel = elevator_vel_sig_.getValue().in(DegreesPerSecond); 
        inputs.elevatorVelocity = MetersPerSecond.of(vel * ManipulatorConstants.Elevator.kMetersPerRev); 

        inputs.elevator1Voltage = elevator_1_vol_sig_.getValue();
        inputs.elevator1Current = elevator_1_current_sig_.getValue();

        inputs.elevator2Voltage = elevator_2_vol_sig_.getValue();
        inputs.elevator2Current = elevator_2_current_sig_.getValue();

        inputs.hallEffectSensor = hall_effect_sensor_.get() ;

        if (Robot.isSimulation()) {
            simulateArm() ;
            simulateElevator() ;
            
            // arm encoder inputs: 
            inputs.rawAbsoluteEncoder = arm_encoder_sim_.get() ;
            inputs.absoluteEncoder = Degrees.of(mapper_.toRobot(inputs.rawAbsoluteEncoder)) ;
        }        
        else {
            // arm encoder inputs: 
            inputs.rawAbsoluteEncoder = encoder_.get();
            inputs.absoluteEncoder = Degrees.of(mapper_.toRobot(inputs.rawAbsoluteEncoder)); 
        }
    }

    public void setArmMotorVoltage(Voltage vol) {
        arm_voltage_ = vol;
        arm_motor_.setControl(new VoltageOut(arm_voltage_));
    }

    public void logArmMotor(SysIdRoutineLog log) {
        log.motor("arm")
            .voltage(arm_voltage_)
            .angularPosition(Revolutions.of(arm_pos_sig_.refresh().getValueAsDouble()))
            .angularVelocity(RevolutionsPerSecond.of(arm_vel_sig_.refresh().getValueAsDouble()));
    }

    public void setElevatorMotorVoltage(double vol) {
        elevator_voltage_ = Volts.of(vol) ;
        elevator_motor_.setControl(new VoltageOut(elevator_voltage_));    
    }

    public void logElevatorMotor(SysIdRoutineLog log) {
        log.motor("elevator")
            .voltage(elevator_voltage_)
            .angularPosition(elevator_pos_sig_.refresh().getValue())
            .angularVelocity(elevator_vel_sig_.refresh().getValue()) ;
    } 

    public void setElevatorTarget(Distance dist) {
        double revs = dist.in(Meters) / ManipulatorConstants.Elevator.kMetersPerRev;
        elevator_motor_.setControl(new MotionMagicVoltage(Revolutions.of(revs)).withSlot(0));
    }

    public void resetPosition() {
        elevator_motor_.setPosition(Degrees.of(0.0));
    }

    public void setArmTarget(Angle angle) {
        arm_motor_.setControl(new MotionMagicVoltage(angle.times(ManipulatorConstants.Arm.kGearRatio)).withSlot(0)); 
    }

    public void syncArmPosition() {
        double enc = encoder_.get() ;
        double angle = mapper_.toRobot(enc) ;
        Angle armAngle = Degrees.of(angle).times(ManipulatorConstants.Arm.kGearRatio) ;
        arm_motor_.setPosition(armAngle) ;
    }

    private void simulateArm() {
        TalonFXSimState st = arm_motor_.getSimState() ;
        st.setSupplyVoltage(RobotController.getBatteryVoltage()) ;

        Voltage mv = st.getMotorVoltageMeasure() ;
        arm_sim_.setInputVoltage(mv.in(Volts)) ;
        arm_sim_.update(0.02) ;
        st.setRawRotorPosition(arm_sim_.getAngularPosition().times(ManipulatorConstants.Arm.kGearRatio)) ;
        st.setRotorVelocity(arm_sim_.getAngularVelocity().times(ManipulatorConstants.Arm.kGearRatio)) ;

        if (encoder_motor_synced_) {
            arm_encoder_sim_.set(mapper_.toEncoder(arm_sim_.getAngularPosition().in(Degrees))) ;
            encoder_motor_synced_ = true ;
        }
    }

    private void simulateElevator() {
        TalonFXSimState st = elevator_motor_.getSimState() ;
        st.setSupplyVoltage(RobotController.getBatteryVoltage()) ;

        Voltage mv = st.getMotorVoltageMeasure() ;
        elevator_sim_.setInputVoltage(mv.in(Volts)) ;
        elevator_sim_.update(0.02) ;
        st.setRawRotorPosition(elevator_sim_.getAngularPosition().times(ManipulatorConstants.Elevator.kGearRatio)) ;
        st.setRotorVelocity(elevator_sim_.getAngularVelocity().times(ManipulatorConstants.Elevator.kGearRatio)) ;
    }    
}
