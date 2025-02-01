package frc.simulator.models;

import edu.wpi.first.hal.simulation.DIODataJNI;
import frc.simulator.engine.SimulationEngine;
import frc.simulator.engine.SimulationModel;
import frc.simulator.utils.BadParameterTypeException;
import frc.simulator.utils.MessageLogger;
import frc.simulator.utils.MessageType;
import frc.simulator.utils.SettingsValue;

public class GrabberModel extends SimulationModel {
    int low_coral_sensor_ ;
    int high_coral_sensor_ ;
    int funnel_coral_sensor_ ;
    int low_algae_sensor_ ;
    int high_algae_sensor_ ;
    
    public GrabberModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    public boolean create(SimulationEngine engine) {
        low_coral_sensor_ = initOne("low-coral-sensor") ;
        high_coral_sensor_ = initOne("high-coral-sensor") ;
        funnel_coral_sensor_ = initOne("funnel-coral-sensor") ;
        low_algae_sensor_ = initOne("low-algae-sensor") ;
        high_algae_sensor_ = initOne("high-algae-sensor") ;

        if (low_coral_sensor_ < 0 || high_coral_sensor_ < 0 || funnel_coral_sensor_ < 0 || low_algae_sensor_ < 0 || high_algae_sensor_ < 0) {
            return false ;
        }

        return true ;
    }
    @Override
    public boolean processEvent(String name, SettingsValue value) {
        boolean ret = false ;
        
        try {
            if (name.equals("low-coral-sensor")) {
                DIODataJNI.setValue(low_coral_sensor_, value.getBoolean());
                ret = true ;
            }
            else if (name.equals("high-coral-sensor")) {
                DIODataJNI.setValue(high_coral_sensor_, value.getBoolean());
                ret = true ;
            }
            else if (name.equals("funnel-coral-sensor")) {
                DIODataJNI.setValue(funnel_coral_sensor_, value.getBoolean());
                ret = true ;
            }
            else if (name.equals("low-algae-sensor")) {
                DIODataJNI.setValue(low_algae_sensor_, value.getBoolean());
                ret = true ;
            }
            else if (name.equals("high-algae-sensor")) {
                DIODataJNI.setValue(high_algae_sensor_, value.getBoolean());
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

    private int initOne(String name) {
        try {
            int sensor = getProperty(name).getInteger() ;
            DIODataJNI.setIsInput(sensor, true);
            DIODataJNI.setValue(sensor, false) ;
            return sensor ;
        }
        catch(Exception ex) {
            MessageLogger.getTheMessageLogger().startMessage(MessageType.Error).add("cannot find property '" + name + "'").endMessage(); 
            return -1 ;
        }
    }
}
