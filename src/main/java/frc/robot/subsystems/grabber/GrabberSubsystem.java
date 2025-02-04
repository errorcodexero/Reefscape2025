package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

    private final GrabberIO io_;
    private final GrabberIOInputsAutoLogged inputs_;

    private boolean has_coral_;
    private boolean has_algae_;

    public GrabberSubsystem(GrabberIO io) {
        io_ = io;
        inputs_ = new GrabberIOInputsAutoLogged();
    }

    public void ejectCoralReef() {
        if (has_coral_) {
            io_.setGrabberTargetVelocity(GrabberConstants.Grabber.Positions.ejectCoralVelocty);
            if (inputs_.coralFrontFallingEdge) {
                has_coral_ = false;
            }
        }
    }

    public void waitForCoral() {
        io_.setGrabberTargetVelocity(GrabberConstants.Grabber.Positions.waitForCoralVelocity);
        if (inputs_.coralFunnelRisingEdge) {
            has_coral_ = true;
        }
    }

    public void holdingCoral() {
        if (inputs_.coralFrontRisingEdge) {
            stopGrabber();
        }
    }

    public void stopGrabber() {
        io_.setGrabberTargetVelocity(0);
    }

    public void setGrabberMotorVoltage(double vol) {
        io_.setGrabberMotorVoltage(vol);
    }

    public boolean hasCoral() {
        return has_coral_;
    }

    public void setHasCoral(boolean b) {
        has_coral_ = b;
    }

    public boolean hasAlgae() {
        return has_algae_;
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.recordOutput("coral", has_coral_);
        Logger.recordOutput("algae", has_algae_);
        Logger.processInputs("grabber", inputs_);
    }

}
