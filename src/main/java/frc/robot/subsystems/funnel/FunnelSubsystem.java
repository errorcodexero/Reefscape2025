package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelSubsystem extends SubsystemBase {

    private static final double kCoralLastSeenTimeout = 3.0 ;

    private final FunnelIO io_; 
    private final FunnelInputsAutoLogged inputs_;
    private Angle target_ ;
    private double lastuppercoral = 0.0 ;
    private double lastlowercoral = 0.0 ;

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

        if (inputs_.coralFunnelUpperSensor) {
            lastuppercoral = Timer.getFPGATimestamp();
        }

        if (inputs_.coralFunnelLowerSensor) {
            lastlowercoral = Timer.getFPGATimestamp();  
        }

        Logger.recordOutput("funnel/seencoral", hasSeenCoral());
        Logger.recordOutput("funnel/target", target_) ;
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
     * Functionally the same as {@link #hasSeenCoral()}, but in the event that you have seen coral,
     * this automatically resets the flag for you.
     * @return Whether or not the Funnel has seen a Coral since the last {@link #resetSeenCoral()}.
     */
    public boolean hasSeenCoral() {
        double now = Timer.getFPGATimestamp() ;

        // if (now - lastuppercoral < kCoralLastSeenTimeout) {
        //     return true ;
        // }

        if (now - lastlowercoral < kCoralLastSeenTimeout) {
            return true ;
        }

        return false;
    }    

    public boolean lowerCoralSensor() {
        return inputs_.coralFunnelLowerSensor;
    }

    public boolean coralFunnelUpperSensor() {
        return inputs_.coralFunnelUpperSensor;
    }
}

