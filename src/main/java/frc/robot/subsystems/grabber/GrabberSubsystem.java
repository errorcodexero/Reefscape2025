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

    public boolean isCoralSeen() {
        return inputs_.risingEdge ;
    }
}
