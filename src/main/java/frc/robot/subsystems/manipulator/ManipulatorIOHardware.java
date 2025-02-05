package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.xerosw.util.EncoderMapper;
import org.xerosw.util.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.controls.Follower; 

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Robot;

public class ManipulatorIOHardware implements ManipulatorIO {

    // Arm related members
    private TalonFX arm_motor_;
    private Voltage arm_volts_ ;
    private StatusSignal<Angle> arm_position_ ;
    private StatusSignal<AngularVelocity> arm_velocity_ ;
    private StatusSignal<Current> arm_current_ ;
    private StatusSignal<Voltage> arm_voltage_ ;
    private DCMotorSim arm_sim_ ;

    private DutyCycleEncoder encoder_ ;
    private EncoderMapper mapper_ ;

    // Elevator related members
    private TalonFX elevator_motor_1_; 
    private TalonFX elevator_motor_2_;
    private Voltage elevator_volts_ ;
    private StatusSignal<Angle> elevator_position_ ;
    private StatusSignal<AngularVelocity> elevator_velocity_ ;
    private StatusSignal<Current> elevator_current_ ;
    private StatusSignal<Voltage> elevator_voltage_ ;
    private ElevatorSim elevator_sim_ ;

    public ManipulatorIOHardware() {
        createArm();
        createElevator();

        TalonFXFactory.checkError(-1, "manipulator-update-frequencies-50", () -> BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50.0),
                        arm_position_,
                        arm_velocity_,
                        arm_current_,
                        arm_voltage_,
                        elevator_position_,
                        elevator_velocity_,
                        elevator_current_,
                        elevator_voltage_)) ;


        arm_motor_.optimizeBusUtilization() ;
        elevator_motor_1_.optimizeBusUtilization() ;
    }

    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.armPosition = arm_position_.refresh().getValue().div(ManipulatorConstants.Arm.kGearRatio) ;
        inputs.armVelocity = arm_velocity_.refresh().getValue().div(ManipulatorConstants.Arm.kGearRatio) ;
        inputs.armRawPosition = arm_position_.refresh().getValue() ;
        inputs.armRawVelocity = arm_velocity_.refresh().getValue() ;
        inputs.armCurrent = arm_current_.refresh().getValue() ;
        inputs.armVoltage = arm_voltage_.refresh().getValue() ;

        inputs.armRawEncoder = encoder_.get() ;
        inputs.armEncoderValue = Degrees.of(mapper_.toRobot(inputs.armRawEncoder)) ;

        inputs.elevatorPosition = Meters.of(elevator_position_.refresh().getValue().in(Revolutions) / ManipulatorConstants.Elevator.kMotorRevsToHeightMeters).plus(ManipulatorConstants.Elevator.kMinHeight) ;
        inputs.elevatorRawPosition = elevator_position_.refresh().getValue() ;
        inputs.elevatorRawVelocity = elevator_velocity_.refresh().getValue() ;
        inputs.elevatorVelocity = MetersPerSecond.of(elevator_velocity_.refresh().getValue().in(RevolutionsPerSecond) / ManipulatorConstants.Elevator.kMotorRevsToHeightMeters) ;
        inputs.elevatorCurrent = elevator_current_.refresh().getValue() ;
        inputs.elevatorVoltage = elevator_voltage_.refresh().getValue() ;

        if (Robot.isSimulation()) {
            simulateArm() ;
            simulateElevator() ;
        }
    }

    // arm related methods
    public void setArmAngle(Angle target) {
        // Convert the target angle to motor rotations based on the gear ratio
        Angle mtarget = target.times(ManipulatorConstants.Arm.kGearRatio) ;

        // Create the control request to go to the desired angle
        ControlRequest ctrl = new MotionMagicVoltage(mtarget).withSlot(0).withEnableFOC(true) ;

        // Tell the motor
        arm_motor_.setControl(ctrl) ;
    }

    public void setArmMotorVoltage(double volts) {
        Logger.recordOutput("Arm/charvolts", volts) ;
        arm_volts_ = Volts.of(volts) ;
        ControlRequest ctrl = new VoltageOut(arm_volts_) ;
        arm_motor_.setControl(ctrl) ;
    }

    public void logArmMotor(SysIdRoutineLog log) {
        Angle pos = arm_position_.refresh().getValue() ;
        AngularVelocity vel = arm_velocity_.refresh().getValue() ;

        log.motor("arm")
            .voltage(arm_volts_)
            .angularPosition(pos)
            .angularVelocity(vel) ;
    }

    // elevator related methods
    public void setElevatorHeight(Distance target) {
        // Compute the distance above the bottom of the elevator travel
        Distance habovezero = target.minus(ManipulatorConstants.Elevator.kMinHeight) ;

        // Convert the distance above the bottom of the elevator travel to motor rotations
        Angle pos = Revolutions.of(habovezero.in(Meters) * ManipulatorConstants.Elevator.kMotorRevsToHeightMeters) ;

        // Create the control request to go to the desired height
        ControlRequest ctrl = new MotionMagicVoltage(pos).withSlot(0).withEnableFOC(true) ;

        // Tell the motor
        elevator_motor_1_.setControl(ctrl) ;
    }

    public void setElevatorMotorVoltage(double volts) {
        elevator_volts_ = Volts.of(volts) ;
        ControlRequest ctrl = new VoltageOut(elevator_volts_) ;
        elevator_motor_1_.setControl(ctrl) ;
    }

    public void logElevatorMotor(SysIdRoutineLog log) {
        Angle pos = elevator_position_.refresh().getValue() ;
        AngularVelocity vel = elevator_velocity_.refresh().getValue() ;

        log.motor("arm")
            .voltage(elevator_volts_)
            .angularPosition(pos)
            .angularVelocity(vel) ;
    }

    private void createArm() {
        TalonFXFactory f = TalonFXFactory.getFactory() ;
        arm_motor_ = f.createTalonFX(ManipulatorConstants.Arm.kMotorCANID, 
                                     ManipulatorConstants.Arm.kInverted,
                                     ManipulatorConstants.Arm.kCurrentLimit) ;

        arm_position_ = arm_motor_.getPosition() ;
        arm_velocity_ = arm_motor_.getVelocity() ;
        arm_current_ = arm_motor_.getSupplyCurrent() ;
        arm_voltage_ = arm_motor_.getMotorVoltage() ;

        // ARM CONFIGS:     
        Slot0Configs arm_pids = new Slot0Configs();
        arm_pids.kP = ManipulatorConstants.Arm.PID.kP; 
        arm_pids.kI = ManipulatorConstants.Arm.PID.kI; 
        arm_pids.kD = ManipulatorConstants.Arm.PID.kD; 
        arm_pids.kV = ManipulatorConstants.Arm.PID.kV; 
        arm_pids.kA = ManipulatorConstants.Arm.PID.kA; 
        arm_pids.kG = ManipulatorConstants.Arm.PID.kG; 
        arm_pids.kS = ManipulatorConstants.Arm.PID.kS; 
        TalonFXFactory.checkError(ManipulatorConstants.Arm.kMotorCANID, "apply", () -> arm_motor_.getConfigurator().apply(arm_pids)) ;

        MotionMagicConfigs armMotionMagicConfigs = new MotionMagicConfigs(); 
        armMotionMagicConfigs.MotionMagicCruiseVelocity = ManipulatorConstants.Arm.MotionMagic.kMaxVelocity.in(RotationsPerSecond) ;
        armMotionMagicConfigs.MotionMagicAcceleration = ManipulatorConstants.Arm.MotionMagic.kMaxAcceleration.in(RotationsPerSecondPerSecond) ;
        armMotionMagicConfigs.MotionMagicJerk = ManipulatorConstants.Arm.MotionMagic.kJerk;
        TalonFXFactory.checkError(ManipulatorConstants.Arm.kMotorCANID, "apply", () -> arm_motor_.getConfigurator().apply(armMotionMagicConfigs)) ;

        encoder_ = new DutyCycleEncoder(ManipulatorConstants.Arm.ThruBoreEncoder.kDutyCyclePin) ; 

        mapper_ = new EncoderMapper(ManipulatorConstants.Arm.ThruBoreEncoder.kRobotMax,
                                    ManipulatorConstants.Arm.ThruBoreEncoder.kRobotMin,
                                    ManipulatorConstants.Arm.ThruBoreEncoder.kEncoderMax,
                                    ManipulatorConstants.Arm.ThruBoreEncoder.kEncoderMin) ;

        mapper_.calibrate(ManipulatorConstants.Arm.ThruBoreEncoder.kRobotCalibrationValue,
                          ManipulatorConstants.Arm.ThruBoreEncoder.kEncoderCalibrationValue) ;

        // For now, force the arm to the zero position when the robot is initialized.
        arm_motor_.setPosition(Rotations.of(0.0)) ;

        if (Robot.isSimulation()) {
            LinearSystem<N2, N1, N2> sys = LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 
                                                                              ManipulatorConstants.Arm.kMOI.in(KilogramSquareMeters), 
                                                                              ManipulatorConstants.Arm.kGearRatio) ;
            arm_sim_ = new DCMotorSim(sys, DCMotor.getKrakenX60Foc(1)) ;
        }
    }

    private void createElevator() {
        TalonFXFactory f = TalonFXFactory.getFactory() ;
        elevator_motor_1_ = f.createTalonFX(ManipulatorConstants.Elevator.kMotorCANID1, 
                                            ManipulatorConstants.Elevator.kInverted,
                                            ManipulatorConstants.Elevator.kCurrentLimit) ;
        elevator_motor_1_.setPosition(Degrees.of(0.0)) ;

        elevator_motor_2_ = f.createTalonFX(ManipulatorConstants.Elevator.kMotorCANID2, 
                                            ManipulatorConstants.Elevator.kInverted,
                                            ManipulatorConstants.Elevator.kCurrentLimit) ;
        elevator_motor_2_.setControl(new Follower(elevator_motor_1_.getDeviceID(), true)) ;

        elevator_position_ = elevator_motor_1_.getPosition() ;
        elevator_velocity_ = elevator_motor_1_.getVelocity() ;
        elevator_current_ = elevator_motor_1_.getSupplyCurrent() ;
        elevator_voltage_ = elevator_motor_1_.getMotorVoltage() ;

        // ELEVATOR CONFIGS:
        Slot0Configs elevator_pids = new Slot0Configs();
        elevator_pids.kP = ManipulatorConstants.Elevator.PID.kP;
        elevator_pids.kI = ManipulatorConstants.Elevator.PID.kI;
        elevator_pids.kD = ManipulatorConstants.Elevator.PID.kD;
        elevator_pids.kV = ManipulatorConstants.Elevator.PID.kV;
        elevator_pids.kA = ManipulatorConstants.Elevator.PID.kA;
        elevator_pids.kG = ManipulatorConstants.Elevator.PID.kG;
        elevator_pids.kS = ManipulatorConstants.Elevator.PID.kS;
        TalonFXFactory.checkError(ManipulatorConstants.Elevator.kMotorCANID1, "apply", () -> elevator_motor_1_.getConfigurator().apply(elevator_pids)) ;

        MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = ManipulatorConstants.Elevator.MotionMagic.kMaxVelocity.in(RotationsPerSecond) ;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = ManipulatorConstants.Elevator.MotionMagic.kMaxAcceleration.in(RotationsPerSecondPerSecond) ;
        elevatorMotionMagicConfigs.MotionMagicJerk = ManipulatorConstants.Elevator.MotionMagic.kJerk ;

        TalonFXFactory.checkError(ManipulatorConstants.Elevator.kMotorCANID1, "apply", () -> elevator_motor_1_.getConfigurator().apply(elevatorMotionMagicConfigs)) ;

        if (Robot.isSimulation()) {
            DCMotor gearbox = DCMotor.getFalcon500Foc(2) ;
            elevator_sim_ = new ElevatorSim(gearbox, ManipulatorConstants.Elevator.kGearRatio, 
                                            ManipulatorConstants.Elevator.kCarriageMass.in(Kilograms), 
                                            ManipulatorConstants.Elevator.kDrumRadius.in(Meters),
                                            ManipulatorConstants.Elevator.kMinHeight.in(Meters),
                                            ManipulatorConstants.Elevator.kMaxHeight.in(Meters),
                                            true, ManipulatorConstants.Elevator.kMinHeight.in(Meters)) ;
        }
    }

    private void simulateArm() {
        TalonFXSimState st = arm_motor_.getSimState() ;
        st.setSupplyVoltage(RobotController.getBatteryVoltage()) ;

        Voltage mv = st.getMotorVoltageMeasure() ;
        arm_sim_.setInputVoltage(mv.in(Volts)) ;
        arm_sim_.update(0.02) ;
        st.setRawRotorPosition(arm_sim_.getAngularPosition().times(ManipulatorConstants.Arm.kGearRatio)) ;
        st.setRotorVelocity(arm_sim_.getAngularVelocity().times(ManipulatorConstants.Arm.kGearRatio)) ;
    }

    private void simulateElevator() {
        TalonFXSimState st = elevator_motor_1_.getSimState() ;
        st.setSupplyVoltage(RobotController.getBatteryVoltage()) ;

        Voltage mv = st.getMotorVoltageMeasure() ;
        elevator_sim_.setInputVoltage(mv.in(Volts)) ;
        elevator_sim_.update(0.02) ;
        st.setRawRotorPosition(elevator_sim_.getPositionMeters() / ManipulatorConstants.Elevator.kMotorRevsToHeightMeters) ;
        st.setRotorVelocity(elevator_sim_.getVelocityMetersPerSecond() / ManipulatorConstants.Elevator.kMotorRevsToHeightMeters) ;
    }
}