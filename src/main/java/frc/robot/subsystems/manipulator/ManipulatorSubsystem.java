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
    private Angle target_angle_ ;
    private Distance target_height_ ;

    private final Alert armDisconnected_ = new Alert("Arm motor failed to configure or is disconnected!", AlertType.kError);
    private final Alert elevator1Disconnected_ = new Alert("Elevator motor 1 failed to configure or is disconnected!", AlertType.kError);
    private final Alert elevator2Disconnected_ = new Alert("Elevator motor 2 failed to configure or is disconnected!", AlertType.kError);

    public ManipulatorSubsystem(ManipulatorIO io) {
        io_ = io; 
        inputs_ = new ManipulatorIOInputsAutoLogged(); 
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Manipulator", inputs_);

        armDisconnected_.set(!inputs_.armReady);
        elevator1Disconnected_.set(!inputs_.elevator1Ready);
        elevator2Disconnected_.set(!inputs_.elevator2Ready);
    }

    public Angle getArmPosition() {
        return inputs_.armPosition ;
    }

    public void setArmPosition(Angle angle) {
        target_angle_ = angle;  
        io_.setArmPosition(angle); 
    }

    public Distance getElevatorPosition() {
        return inputs_.elevatorPosition; 
    }

    public void setElevatorPosition(Distance dist) {
        target_height_ = dist ;
        io_.setElevatorPosition(dist); 
    }

    public boolean doesCrossKZ(Angle current, Angle target) {
        Angle keepout_min = ManipulatorConstants.Keepout.kKeepoutMinAngle; 
        Angle keepout_max = ManipulatorConstants.Keepout.kKeepoutMaxAngle; 

        if(current.lt(keepout_min) && target.gt(keepout_max)) {
            return true; 
        } else if(current.gt(keepout_max) && target.lt(keepout_min)) {
            return true; 
        }
        return false;
    }

    public boolean isElevAtTarget() {
        if(inputs_.elevatorPosition.isNear(target_height_, ManipulatorConstants.Elevator.kPosTolerance)) {
            return true; 
        }
        return false; 
    }

    public boolean isArmAtTarget() {
        if(inputs_.armPosition.isNear(target_angle_, ManipulatorConstants.Arm.kPosTolerance)) {
            return true; 
        }
        return false; 
    }
}