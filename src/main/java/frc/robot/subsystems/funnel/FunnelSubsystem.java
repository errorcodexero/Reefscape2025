package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelSubsystem extends SubsystemBase {

    private final FunnelIO io_; 
    private final FunnelInputsAutoLogged inputs_;
    private Angle target_ ;
    private double lastcoral = 0.0 ;

    private final Alert disconnectedAlert_ = new Alert("Funnel motor is disconnected or failed to initialize!", AlertType.kError);

    public FunnelSubsystem(FunnelIO io) {
        io_ = io;
        inputs_ = new FunnelInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Funnel", inputs_);

        disconnectedAlert_.set(!inputs_.funnelReady);

        if (!inputs_.coralFunnelSensor || inputs_.coralFunnelFallingEdge) {
            lastcoral = Timer.getTimestamp();
        }

        Logger.recordOutput("funnel/seencoral", hasSeenCoral());
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
     * Whether or not the Funnel has seen a Coral in the last few seconds.
     */
    public boolean hasSeenCoral() {
        if (Timer.getTimestamp() - lastcoral < 3.0) {
            return true ;
        }

        return false;
    }    
}

