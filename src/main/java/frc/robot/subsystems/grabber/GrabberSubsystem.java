package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    ///////////////////////////
    // CoralFront Sensor States
    ///////////////////////////

    public boolean coralFrontRising() {
        return inputs_.coralFrontRisingEdge;
    }

    public boolean coralFrontFalling() {
        return inputs_.coralFrontFallingEdge;
    }

    //////////////////////////
    // CoralBack Sensor States
    //////////////////////////

    public boolean coralBackRising() {
        return inputs_.coralBackRisingEdge;
    }

    public boolean coralBackFalling() {
        return inputs_.coralBackFallingEdge;
    }

    ////////////////////////////
    // CoralFunnel Sensor States 
    ////////////////////////////

    public boolean coralFunnelRising() {
        return inputs_.coralFunnelRisingEdge;
    }

    public boolean coralFunnelFalling() {
        return inputs_.coralFunnelFallingEdge;
    }

    ///////////////////////////
    // AlgaeUpper Sensor States
    ///////////////////////////

    public boolean AlgaeUpperRising() {
        return inputs_.algaeUpperRisingEdge;
    }

    public boolean AlgaeUpperFalling() {
        return inputs_.algaeUpperFallingEdge;
    }

    ///////////////////////////
    // AlgaeLower Sensor States
    ///////////////////////////

    public boolean AlgaeLowerRising() {
        return inputs_.algaeLowerRisingEdge;
    }

    public boolean AlgaeLowerFalling() {
        return inputs_.algaeLowerFallingEdge;
    }

}
