package frc.robot.subsystems.algaeManipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeManipulator extends SubsystemBase{
    private AlgaeManipulatorIO io_; 
    private AlgaeManipulatorIOInputsAutoLogged inputs_ = new AlgaeManipulatorIOInputsAutoLogged();

     public AlgaeManipulator(AlgaeManipulatorIO io){
        io_ = io;
     }

     @Override
     public void periodic(){
        io_.updateInputs(inputs_);
     }
}

