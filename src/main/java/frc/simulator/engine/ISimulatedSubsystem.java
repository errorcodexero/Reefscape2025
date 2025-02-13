package frc.simulator.engine;

import org.xerosw.util.SettingsValue;

public interface ISimulatedSubsystem {
    SettingsValue getProperty(String name) ;
}
