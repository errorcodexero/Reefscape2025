package frc.robot.subsystems.algaeManipulator;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class AlgaeManipulatorIOHardware implements AlgaeManipulatorIO {
    private static final Time kRobotLoop = Milliseconds.of(20);

    TalonFX hinge_motor_;
    TalonFX roller_motor_;

    private DCMotorSim hinge_sim_;
    private DCMotorSim roller_sim_;

    private double hinge_voltage_;
    private double roller_voltage_;

    public void AlgaeManipulatorIO() {

        // create motors

        // configure motors

        // configure PID 

        // set up motion magic


        if (RobotBase.isSimulation()) {
            LinearSystem<N2, N1, N2> hinge = LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 
                                                                              AlgaeManipulatorConstants.kMOI.in(KilogramSquareMeters), 
                                                                              AlgaeManipulatorConstants.kGearRatio) ;
            LinearSystem<N2, N1, N2> rollers = LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 
                                                                              AlgaeManipulatorConstants.kMOI.in(KilogramSquareMeters), 
                                                                              AlgaeManipulatorConstants.kGearRatio) ;
            hinge_sim_ = new DCMotorSim(hinge, DCMotor.getKrakenX60Foc(1)) ;
            roller_sim_ = new DCMotorSim(rollers, DCMotor.getKrakenX60Foc(1)) ;            
        }
    }


    // ---------------------------
    // Hinge Methods
    // ---------------------------

    public void setHingeMotorVoltage(double vol) {
        hinge_voltage_ = vol;
        hinge_motor_.setControl(new VoltageOut(hinge_voltage_));
    }

    public double getHingeMotorVoltage(){
        return hinge_voltage_;
    }

    public void logHingeMotor(SysIdRoutineLog log) {
    }


    // ---------------------------
    // Roller Methods
    // ---------------------------

    public void setRollerMotorVoltage(double vol) {
        roller_voltage_ = vol;
        roller_motor_.setControl(new VoltageOut(roller_voltage_));
    }

    public double getRollerMotorVoltage(){
        return roller_voltage_;
    }

    public void logRollerMotor(SysIdRoutineLog log) {
    }



    public void simulate() {
        TalonFXSimState st, st1;
        Voltage v, v1;
        
        st = hinge_motor_.getSimState();
        st.setSupplyVoltage(RobotController.getBatteryVoltage());
        v = st.getMotorVoltageMeasure();
        hinge_sim_.setInputVoltage(v.in(Volts));
        hinge_sim_.update(kRobotLoop.in(Seconds));
        st.setRawRotorPosition(hinge_sim_.getAngularPosition().times(AlgaeManipulatorConstants.kGearRatio));
        st.setRotorVelocity(hinge_sim_.getAngularVelocity().times(AlgaeManipulatorConstants.kGearRatio));

        st1 = roller_motor_.getSimState();
        st1.setSupplyVoltage(RobotController.getBatteryVoltage());
        v1 = st1.getMotorVoltageMeasure();
        roller_sim_.setInputVoltage(v1.in(Volts));
        roller_sim_.update(kRobotLoop.in(Seconds));
        st1.setRawRotorPosition(roller_sim_.getAngularPosition().times(AlgaeManipulatorConstants.kGearRatio));
        st1.setRotorVelocity(roller_sim_.getAngularVelocity().times(AlgaeManipulatorConstants.kGearRatio));
    }

    @Override
    public void updateInputs(AlgaeManipulatorIOInputsAutoLogged inputs_) {

    }

    // Check Error Function Below?

}
