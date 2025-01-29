package frc.robot.subsystems.manipulator;

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
}
