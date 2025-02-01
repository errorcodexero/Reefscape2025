package frc.robot.subsystems.manipulator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class ManipulatorSubsystem extends SubsystemBase{
    private final ManipulatorIO io_; 
    private final ManipulatorIOInputsAutoLogged inputs_;  

    public ManipulatorSubsystem(ManipulatorIO io){
        io_ = io; 
        inputs_ = new ManipulatorIOInputsAutoLogged(); 
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
    }

    public void setArmPosition(Angle angle){
        io_.setArmPosition(angle); 
    }

    public void setElevatorPosition(Distance dist){
        io_.setElevatorPosition(dist); 
    }

    public boolean doesCrossKZ(Angle current, Angle target){
        Angle keepout = ManipulatorConstants.Keepout.kKeepoutAngle; 
        if(current.lt(keepout) && target.gt(keepout)){
            return true; 
        } else if(current.gt(keepout) && target.lt(keepout)){
            return true; 
        }
        return false;
    }

    public boolean isElevAtTarget(Angle current, Angle target){
        return false; 
    }
}