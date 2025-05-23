package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.xerosw.util.XeroTimer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class GrabberSubsystem extends SubsystemBase {

    private static Angle kHoldOffset = Rotations.of(-0.4) ;

    private enum CollectState {
        COLLECTING,
        DELAY,
        BACKING_UP,
        HOLDING,
        IDLE
    }

    private final GrabberIO io_;
    private final GrabberIOInputsAutoLogged inputs_;
    private CollectState collect_state_ ;
    private XeroTimer timer_ ;
    private Angle grabber_target_ ;

    private final Alert disconnectedAlert = new Alert("Grabber motor was not initialized correctly!", AlertType.kError);

    public GrabberSubsystem(GrabberIO io) {
        io_ = io;
        inputs_ = new GrabberIOInputsAutoLogged();
        collect_state_ = CollectState.IDLE ;
        timer_ = new XeroTimer(Milliseconds.of(20)) ;
        grabber_target_ = Rotations.zero() ;
    }

    public void collecting() {
        collect_state_ = CollectState.COLLECTING ;
    }

    public void holding() {
        collect_state_ = CollectState.HOLDING ;
    }

    public void idle() {
        collect_state_ = CollectState.IDLE ;
    }

    public int coralOnFloor() {
        return inputs_.numberOfCoral;
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Grabber", inputs_);
        Logger.recordOutput("grabber/target", grabber_target_) ;

        disconnectedAlert.set(!inputs_.grabberReady);

        Logger.recordOutput("grabber/state", collect_state_.toString()) ;
        switch(collect_state_) {
            case COLLECTING:
                if (inputs_.coralSensor) {
                    setGrabberMotorVoltage(Volts.zero()) ;
                    collect_state_ = CollectState.DELAY ;
                    timer_.start() ;
                }
                break ;

            case DELAY:
                if (timer_.isExpired()) {
                    collect_state_ = CollectState.BACKING_UP ;
                    setGrabberMotorVoltage(Volts.of(-1.0)) ;
                }
                break ;

            case BACKING_UP:
                if (!inputs_.coralSensor) {
                    setGrabberTargetPosition(inputs_.grabberPosition.plus(kHoldOffset)) ;
                    collect_state_ = CollectState.HOLDING ;
                }
                break ;

            case HOLDING:
                if (inputs_.coralSensor) {
                    collect_state_ = CollectState.BACKING_UP ;
                    setGrabberMotorVoltage(Volts.of(-1.0)) ;                    
                }
                break ;

            case IDLE:
                break ;
        }
    }

    public void setGrabberTargetPosition(Angle pos) {
        grabber_target_ = pos ;
        io_.setGrabberTargetPosition(pos);
    }

    //////////////////
    // Grabber Methods
    //////////////////

    public void stopGrabber() {
        io_.setGrabberMotorVoltage(Volts.zero()) ;
    }

    public Command stopGrabberCommand() {
        return runOnce(this::stopGrabber);
    }

    public void setGrabberMotorVoltage(Voltage vol) {
        io_.setGrabberMotorVoltage(vol);
    }

    public Command setVoltageCommand(Voltage vol) {
        return runOnce(() -> setGrabberMotorVoltage(vol));
    }

    ///////////////////////////
    // Coral Sensor State
    ///////////////////////////

    public boolean coralSensor() {
        return inputs_.coralSensor;
    }

    public boolean hasSeenCoral() {
        return coralSensor() ;
    }

    ///////////////////////////
    // Algae Sensor State
    ///////////////////////////

    public boolean algaeSensor() {
        return inputs_.algaeSensor1;
    }

    public boolean hasAlgae() {
        return !inputs_.algaeSensor1 || !inputs_.algaeSensor2 ;
    }

    ///////////////////////////
    // SysId Routines
    ///////////////////////////
    ///
    public Command grabberSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return grabberIdRoutine().quasistatic(dir);
    }

    public Command grabberSysIdDynamic(SysIdRoutine.Direction dir) {
        return grabberIdRoutine().dynamic(dir);
    }

    private SysIdRoutine grabberIdRoutine() {
        Voltage step = Volts.of(7);
        Time to = Seconds.of(10.0);
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null);

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                (volts) -> io_.setGrabberMotorVoltage(volts),
                (log) -> io_.logGrabberMotor(log),
                this);

        return new SysIdRoutine(cfg, mfg);
    }
}
