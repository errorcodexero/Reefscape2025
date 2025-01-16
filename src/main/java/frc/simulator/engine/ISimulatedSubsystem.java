package frc.simulator.engine;

import frc.simulator.utils.SettingsValue;

public interface ISimulatedSubsystem {
    SettingsValue getProperty(String name) ;
}
