package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

    private GamePieceLocation gp_ ;
    private final GrabberIO io_; 
    private final GrabberIOInputsAutoLogged inputs_; 

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
    }

    public void setGrabberVelocity(AngularVelocity vel) {
        io_.setGrabberVelocity(vel);
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
}
