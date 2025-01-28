package frc.robot.subsystems.manipulator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class ManipulatorSubsystem extends SubsystemBase{
    private final ManipulatorIO io_; 
    private final ManipulatorIOInputsAutoLogged inputs_;
    private Angle target_arm_position_ ;
    private Distance target_elevator_position_ ;

    public ManipulatorSubsystem(ManipulatorIO io){
        io_ = io; 
        inputs_ = new ManipulatorIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
    }

    public Angle getCurrentArmAngle() {
        return inputs_.armPosition ;
    }

    public void setArmAngleTarget(Angle target) {
        io_.setArmAngle(target);
    }

    public Distance getCurrentElevatorHeight() {
        return inputs_.elevatorPosition ;
    }

    public void setElevatorHeightTarget(Distance target) {
        io_.setElevatorHeight(target);
    }

    public boolean isArmAtTarget() {
        return inputs_.armPosition.isNear(target_arm_position_, ManipulatorConstants.Goto.kArmTolerance) ;
    }

    public boolean isElevatorAtTarget() {
        return inputs_.elevatorPosition.isNear(target_elevator_position_, ManipulatorConstants.Goto.kElevatorTolerance) ;
    }
}
