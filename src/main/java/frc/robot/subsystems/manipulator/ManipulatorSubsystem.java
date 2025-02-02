package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class ManipulatorSubsystem extends SubsystemBase{
    private final ManipulatorIO io_; 
    private final ManipulatorIOInputsAutoLogged inputs_;  

    private final Alert armDisconnected_ = new Alert("Arm motor failed to configure or is disconnected!", AlertType.kError);
    private final Alert elevatorDisconnected_ = new Alert("Elevator motor failed to configure or is disconnected!", AlertType.kError);

    public ManipulatorSubsystem(ManipulatorIO io) {
        io_ = io; 
        inputs_ = new ManipulatorIOInputsAutoLogged(); 
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Manipulator", inputs_);

        armDisconnected_.set(!inputs_.armReady);
        elevatorDisconnected_.set(!inputs_.elevatorReady);
    }

    public void setArmPosition(Angle angle) {
        io_.setArmPosition(angle); 
    }

    public void setElevatorPosition(Distance dist) {
        io_.setElevatorPosition(dist); 
    }

    public boolean doesCrossKZ(Angle current, Angle target) {
        Angle keepout_min = ManipulatorConstants.Keepout.kKeepoutMinAngle; 
        Angle keepout_max= ManipulatorConstants.Keepout.kKeepoutMaxAngle; 

        if(current.lt(keepout_min) && target.gt(keepout_max)) {
            return true; 
        } else if(current.gt(keepout_max) && target.lt(keepout_min)) {
            return true; 
        }
        return false;
    }

    public boolean isElevAtTarget(Distance current, Distance target) {
        if(current.isNear(target, ManipulatorConstants.Elevator.kPosTolerance)) {
            return true; 
        }
        return false; 
    }

    public boolean isArmAtTarget(Angle current, Angle target) {
        if(current.isNear(target, ManipulatorConstants.Arm.kPosTolerance)) {
            return true; 
        }
        return false; 
    }
}