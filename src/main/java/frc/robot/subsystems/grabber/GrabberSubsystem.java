package frc.robot.subsystems.grabber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

    private final GrabberIO io_; 
    private final GrabberIOInputsAutoLogged inputs_; 

    public GrabberSubsystem(GrabberIO io){
        io_ = io; 
        inputs_ = new GrabberIOInputsAutoLogged(); 
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
    }

    public void setGrabberVelocity(AngularVelocity vel) {
        io_.setGrabberVelocity(vel);
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
