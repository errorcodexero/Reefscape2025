package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class ManipulatorSubsystem extends SubsystemBase{
    private final ManipulatorIO io_; 
    private final ManipulatorIOInputsAutoLogged inputs_;  

    public ManipulatorSubsystem(ManipulatorIO io) {
        io_ = io; 
        inputs_ = new ManipulatorIOInputsAutoLogged(); 
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Manipulator", inputs_);
    }

    public void setArmPosition(Angle angle) {
        io_.setArmPosition(angle); 
    }

    public void setElevatorPosition(Distance dist) {
        io_.setElevatorPosition(dist); 
    }

    public Angle getArmPosition(){
        return inputs_.armPosition; 
    }

    public Distance getElevatorPosition(){
        return inputs_.elevatorPosition; 
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