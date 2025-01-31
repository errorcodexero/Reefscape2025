package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

    private final GrabberIO io_;
    private final GrabberIOInputsAutoLogged inputs_;

    private final Alert notReadyAlert_ = new Alert("Grabber motor is not connected or was not initialized!", AlertType.kError);

    public GrabberSubsystem(GrabberIO io) {
        io_ = io;
        inputs_ = new GrabberIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Grabber", inputs_);

        notReadyAlert_.set(!inputs_.grabberReady); // Alert if motor is not ready.
    }

}
