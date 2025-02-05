package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class GrabberSubsystem extends SubsystemBase {

    private GamePieceLocation gp_ ;
    private final GrabberIO io_; 
    private final GrabberIOInputsAutoLogged inputs_; 
    private Angle target_ ;

    public GrabberSubsystem(GrabberIO io){
        io_ = io; 
        inputs_ = new GrabberIOInputsAutoLogged(); 
        gp_ = GamePieceLocation.None ;
    }

    public void setGP(GamePieceLocation gp) {
        gp_ = gp ;
    }

    public GamePieceLocation gp() {
        return gp_ ;
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Grabber", inputs_);
        Logger.recordOutput("Grabber/rawvel", inputs_.grabberRawVelocity.in(RotationsPerSecond)) ;
    }

    public Angle getPosition() {
        return inputs_.grabberPosition ;
    }

    public void setGrabberVelocity(AngularVelocity vel) {
        io_.setGrabberVelocity(vel);
    }

    public void setGrabberPosition(Angle target) {
        target_ = target ;
        io_.setGrabberPosition(target);
    }

    public boolean isAtTarget() {
        return inputs_.grabberPosition.isNear(target_, GrabberConstants.Grabber.kTolerance) ;
    }

    public void setGrabberVoltage(Voltage v) {
        io_.setGrabberMotorVoltage(v.in(Volts)) ;
    }

    public boolean isCoralSeenHighRisingEdge() {
        return inputs_.coralSensorHighRisingEdge ;
    }

    public boolean isCoralSeenLowRisingEdge() {
        return inputs_.coralSensorLowRisingEdge ;
    }

    public boolean isCoralSeenLowFallingEdge() {
        return inputs_.coralSensorLowFallingEdge ;
    }

    public boolean isCoralSeenFunnelRisingEdge() {
        return inputs_.coralSensorFunnelRisingEdge ;
    }

    public boolean isAlgaeHighSeenRisingEdge() {
        return inputs_.algaeSensorRisingEdgeHigh ;
    }

    public boolean isAlgaeLowSeenRisingEdge() {
        return inputs_.algaeSensorRisingEdgeLow ;
    }

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
