package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.Robot;

public class ClimberIOSim extends ClimberIOHardware {
    private final DCMotorSim climberSim;
    private final DutyCycleEncoderSim encoderSim;

    public ClimberIOSim() throws Exception {
        LinearSystem<N2, N1, N2> sys = LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 
            ClimberConstants.Climber.kMOI.in(KilogramSquareMeters), 
            ClimberConstants.Climber.kGearRatio
        );

        climberSim = new DCMotorSim(sys, DCMotor.getKrakenX60Foc(1));

        encoderSim = new DutyCycleEncoderSim(encoder_);
        encoderSim.set(robotToEncoder(ClimberConstants.Climber.kStartAbsEncoderAngle.in(Degrees)));
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        TalonFXSimState st = climber_motor_.getSimState();
        st.setSupplyVoltage(RobotController.getBatteryVoltage());

        Voltage mv = st.getMotorVoltageMeasure();
        climberSim.setInputVoltage(mv.in(Volts));
        climberSim.update(Robot.defaultPeriodSecs);

        st.setRawRotorPosition(climberSim.getAngularPosition().times(ClimberConstants.Climber.kGearRatio));
        st.setRotorVelocity(climberSim.getAngularVelocity().times(ClimberConstants.Climber.kGearRatio));

        super.updateInputs(inputs);
        inputs.absEncoderRawValue = encoderSim.get();
        inputs.absEncoderValue = Degrees.of(encoderToRobot(encoderSim.get()));
    }

}
