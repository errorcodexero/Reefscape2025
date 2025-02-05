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

    private boolean has_coral_;

    public Funnel(FunnelIO io) {
        io_ = io; 
        inputs_ = new FunnelInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Manipulator", inputs_);

        disconnectedAlert_.set(!inputs_.funnelReady);

        has_coral_ = inputs_.coralFunnelRisingEdge; // TODO: Figure out whether or not we have a coral, is this sufficient?s
    }
    
    public void runPosition(Angle angle) {
        io_.setPosition(angle);
    }

    public boolean hasCoral() {
        return has_coral_;
    }

}
