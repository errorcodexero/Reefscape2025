package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Degrees;
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
        Logger.recordOutput("Grabber/target", target_);
        Logger.recordOutput("Grabber/ready", isAtTarget()) ;
    }

    public Angle getPosition() {
        return inputs_.grabberPosition ;
    }

    public void setGrabberTargetVelocity(AngularVelocity vel) {
        io_.setGrabberTargetVelocity(vel);
    }

    public void setGrabberTargetPosition(Angle target) {
        target_ = target ;
        io_.setGrabberTargetPosition(target);
    }

    public boolean isAtTarget() {
        if (target_ == null) {
            return false ;
        }

        Logger.recordOutput("Grabber/test-pos", inputs_.grabberPosition.in(Degrees)) ;
        Logger.recordOutput("Grabber/test-target", target_.in(Degrees)) ;
        return inputs_.grabberPosition.isNear(target_, GrabberConstants.Grabber.kTolerance) ;
    }

    public void setGrabberVoltage(Voltage v) {
        io_.setGrabberMotorVoltage(v.in(Volts)) ;
    }

    public Angle getGrabberPositionAtCoralLowEdge() {
        return inputs_.grabberPositionCoralSensorLowEdge ;
    }
    
    public Angle getGrabberPositionAtCoralHighEdge() {
        return inputs_.grabberPositionCoralSensorHighEdge ;
    }

    public boolean coralHighSensorRisingEdge() {
        return inputs_.coralHighRisingEdge ;
    }

    public boolean coralHighSensorFallingEdge() {
        return inputs_.coralHighFallingEdge ;
    }    

    public boolean coralLowSensorRisingEdge() {
        return inputs_.coralSensorLowRisingEdge ;
    }

    public boolean coralLowSensorFallingEdge() {
        return inputs_.coralSensorLowFallingEdge ;
    }

    public boolean coralFunnelRisingEdge() {
        return inputs_.coralFunnelRisingEdge ;
    }

    public boolean algaeHighRisingEdge() {
        return inputs_.algaeHighRisingEdge ;
    }

    public boolean algaeLowRisingEdge() {
        return inputs_.algaeLowRisingEdge ;
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
