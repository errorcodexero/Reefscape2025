package frc.robot.subsystems.algaeManipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.simulator.engine.ISimulatedSubsystem;
import frc.simulator.utils.SettingsValue;

public class AlgaeManipulator extends SubsystemBase implements ISimulatedSubsystem{
    private AlgaeManipulatorIO io_; 
    private AlgaeManipulatorIOInputsAutoLogged inputs_ = new AlgaeManipulatorIOInputsAutoLogged();

     public AlgaeManipulator(AlgaeManipulatorIO io){
        io_ = io;
     }

     @Override
     public void periodic(){
        io_.updateInputs(inputs_);
     }

    @Override
    public SettingsValue getProperty(String name) {
        return null;
    }
}

