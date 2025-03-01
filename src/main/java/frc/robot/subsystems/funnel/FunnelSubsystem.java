package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelSubsystem extends SubsystemBase {

    private final FunnelIO io_; 
    private final FunnelInputsAutoLogged inputs_;
    private Angle target_ ;

    private final Alert disconnectedAlert_ = new Alert("Funnel motor is disconnected or failed to initialize!", AlertType.kError);

    private boolean hasSeenCoral_;

    public FunnelSubsystem(FunnelIO io) {
        io_ = io;
        inputs_ = new FunnelInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Funnel", inputs_);

        disconnectedAlert_.set(!inputs_.funnelReady);

        hasSeenCoral_ = !inputs_.coralFunnelSensor || inputs_.coralFunnelFallingEdge;
    }
    
    public void setTargetPosition(Angle v) {
        target_ = v ;
        io_.setTargetPosition(v) ;
    }

    public boolean isAtTarget() {
        if (target_ == null)
            return false ;

        return inputs_.funnelPosition.isNear(target_, FunnelConstants.kTolerence) ;
    }

    /**
     * Resets the value returned by {@link #hasSeenCoral()}. (If the Funnel has seen Coral since a call of this method)
     * This is intended to be called at the end of a collect cycle.
     */
    public void resetSeenCoral() {
        hasSeenCoral_ = false;
    }

    /**
     * Finds out if the Funnel has seen Coral since the last {@link #resetSeenCoral()}.
     * @return Whether or not the Funnel has seen a Coral since the last {@link #resetSeenCoral()}.
     * @implNote A convenience method that automatically resets this flag for you, is {@link #hasSeenCoralWithReset()}
     */
    public boolean hasSeenCoral() {
        return hasSeenCoral_;
    }

    /**
     * Functionally the same as {@link #hasSeenCoral()}, but in the event that you have seen coral,
     * this automatically resets the flag for you.
     * @return Whether or not the Funnel has seen a Coral since the last {@link #resetSeenCoral()}.
     */
    public boolean hasSeenCoralWithReset() {
        if (hasSeenCoral()) {
            resetSeenCoral();
            return true;
        }

        return false;
    }
    
}

