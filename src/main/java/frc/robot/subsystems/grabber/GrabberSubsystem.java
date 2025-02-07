package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class GrabberSubsystem extends SubsystemBase {
    
    private final GrabberIO io_;
    private final GrabberIOInputsAutoLogged inputs_;
    
    private boolean has_coral_;
    private boolean has_algae_;

    private final Alert disconnectedAlert = new Alert("Grabber motor was not initialized correctly!", AlertType.kError);
    
    public GrabberSubsystem(GrabberIO io) {
        io_ = io;
        inputs_ = new GrabberIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Grabber", inputs_);

        disconnectedAlert.set(!inputs_.grabberReady);
    
        Logger.recordOutput("Grabber/HasCoral", has_coral_);
        Logger.recordOutput("Grabber/HasAlgae", has_algae_);
    }

    //////////////////
    // Grabber Methods
    //////////////////

    public void setGrabberTargetVelocity(double vel) {
        io_.setGrabberTargetVelocity(vel);
    }

    public void stopGrabber() {
        io_.setGrabberTargetVelocity(0);
    }

    public void setGrabberMotorVoltage(double vol) {
        io_.setGrabberMotorVoltage(vol);
    }

    ///////////////////
    // Gamepiece States
    ///////////////////

    public boolean hasCoral() {
        return has_coral_;
    }

    public void setHasCoral(boolean b) {
        has_coral_ = b;
    }

    public boolean hasAlgae() {
        return has_algae_;
    }

    public void setHasAlgae(boolean b) {
        has_algae_ = b;
    }

    ///////////////////////////
    // Coral Sensor State
    ///////////////////////////

    public boolean coralRising() {
        return inputs_.coralRisingEdge;
    }

    public boolean coralFalling() {
        return inputs_.coralFallingEdge;
    }

    ///////////////////////////
    // Algae Sensor State
    ///////////////////////////

    public boolean AlgaeRising() {
        return inputs_.algaeRisingEdge;
    }

    public boolean AlgaeFalling() {
        return inputs_.algaeFallingEdge;
    }

    ///////////////////////////
    // SysId Routines
    ///////////////////////////
    /// 
    public Command grabberSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return grabberIdRoutine().quasistatic(dir) ;
    }

    public Command grabberSysIdDynamic(SysIdRoutine.Direction dir) {
        return grabberIdRoutine().dynamic(dir) ;
    }    

    private SysIdRoutine grabberIdRoutine() {
        Voltage step = Volts.of(7) ;
        Time to = Seconds.of(10.0) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setGrabberMotorVoltage(volts.magnitude()),
                                        (log) -> io_.logGrabberMotor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    }  
}
