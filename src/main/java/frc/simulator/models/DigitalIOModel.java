package frc.simulator.models;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.hal.simulation.DIODataJNI;
import frc.simulator.engine.SimulationEngine;
import frc.simulator.engine.SimulationModel;
import frc.simulator.utils.MessageLogger;
import frc.simulator.utils.MessageType;
import frc.simulator.utils.SettingsValue;

public class DigitalIOModel extends SimulationModel {
    private Map<String, Integer> diomap_ ;

    public DigitalIOModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;

        diomap_ = new HashMap<>() ;
    }

    public boolean create(SimulationEngine engine) {
        Set<String> keys = getPropertyNames() ;
        for(String dioname : keys) {
            if (!dioname.endsWith("-init")) {
                SettingsValue v = getProperty(dioname) ;
                if (v.isInteger()) {
                    try {
                        int channel = v.getInteger() ;
                        boolean value = false ;
                        if (hasProperty(dioname + "-init")) {
                            value = getProperty(dioname + "-init").getBoolean() ;
                        }
                        DIODataJNI.setIsInput(channel, true) ;
                        DIODataJNI.setValue(channel, value) ;
                        diomap_.put(dioname, channel) ;
                    }
                    catch(Exception ex) {
                        MessageLogger.getTheMessageLogger().startMessage(MessageType.Error).add("cannot find property '" + dioname + "'").endMessage(); 
                        return false ;
                    }
                }
            }
        }
        return true ;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        boolean ret = false ;
        
        try {
            if (diomap_.containsKey(name)) {
                int channel = diomap_.get(name) ;                
                DIODataJNI.setValue(channel, value.getBoolean());
                ret = true ;
            }
        }
        catch(Exception ex) {
            MessageLogger logger = MessageLogger.getTheMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("time", getEngine().getSimulationTime());
            logger.add("event", name) ;
            logger.add("- expected boolean value, but got " + value.toString()) ;
            logger.endMessage();
        }
        return ret ;
    }
    @Override
    public void run(double dt) {
    }
}
