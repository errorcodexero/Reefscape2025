package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {

    private final FunnelIO io_; 
    private final FunnelInputsAutoLogged inputs_;

    private final Alert disconnectedAlert_ = new Alert("Funnel motor is disconnected or failed to initialize!", AlertType.kError);

    private boolean hasSeenCoral_;

    public Funnel(FunnelIO io) {
        io_ = io; 
        inputs_ = new FunnelInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Manipulator", inputs_);

        disconnectedAlert_.set(!inputs_.funnelReady);

        hasSeenCoral_ = inputs_.coralFunnelSensor || inputs_.coralFunnelRisingEdge;
    }
    
    public void runPosition(Angle angle) {
        io_.setPosition(angle);
    }

    /**
     * Resets the value returned by {@link #hasSeenCoral()}. (If the Funnel has seen Coral since a call of this method)
     * This is intended to be called at the end of a collect cycle.
     */
    public void resetSeenCoral() {
        hasSeenCoral_ = false;
    }

    /**
     * Whether or not the Funnel has seen a Coral since the last {@link #resetSeenCoral()}.
     * @return
     */
    public boolean hasSeenCoral() {
        return hasSeenCoral_;
    }

}
