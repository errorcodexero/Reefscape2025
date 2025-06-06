package frc.simulator.engine;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import org.xerosw.util.BadParameterTypeException;
import org.xerosw.util.MessageLogger;
import org.xerosw.util.MessageType;
import org.xerosw.util.SettingsValue;


public abstract class SimulationModel {
    
    private SimulationEngine engine_ ;
    private String model_ ;
    private String instance_ ;
    private Map<String, SettingsValue> props_ ;
    private boolean created_ ;
    private int logger_id_ ;
    private boolean warned_not_run_ ;
    
    public SimulationModel(SimulationEngine engine, String model, String instance) {
        engine_ = engine ;
        model_ = model ;
        instance_ = instance ;
        created_ = false ;
        warned_not_run_ = false ;

        props_ = new HashMap<String, SettingsValue>() ;

        logger_id_ = engine.getMessageLogger().registerSubsystem(model + "_model") ;
    }

    public boolean warnedNotRun() {
        return warned_not_run_ ;
    }

    public void setWarnedNotRun() {
        warned_not_run_ = true ;
    }

    public String statusString() {
        return "" ;
    }

    public String getModelName() {
        return model_ ;
    }

    public String getInstanceName() {
        return instance_ ;
    }

    public abstract boolean create(SimulationEngine engine) throws Exception ;
    public abstract void run(double dt) ;
    public abstract boolean processEvent(String name, SettingsValue value) ;
    public void startCycle()  {
    }

    public void endCycle() {
    }

    public boolean hasProperty(String name) {
        return props_.containsKey(name) ;
    }

    public void setProperty(String name, SettingsValue value) {
        props_.put(name, value) ;
    }

    public SettingsValue getProperty(String name) {
        return props_.get(name) ;
    }

    public Set<String> getPropertyNames() {
        return props_.keySet() ;
    }

    public double getRobotTime() {
        return engine_.getSimulationTime();
    }

    public SimulationEngine getEngine() {
        return engine_ ;
    }
    
    public boolean isCreated() {
        return created_ ;
    }

    protected void setCreated() {
        created_ = true ;
    }

    protected int getLoggerID() {
        return logger_id_ ;
    }
    
    protected int getIntProperty(String name) throws Exception {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!hasProperty(name)) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted(name);
            logger.endMessage();
            throw new Exception("getIntProperty failed") ;
        }

        SettingsValue value = getProperty(name) ;
        if (!value.isInteger()) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" property ").addQuoted(name).add(" is not an integer");
            logger.endMessage();   
            throw new Exception("getIntProperty failed") ;         
        }

        return value.getInteger() ;
    }

    protected boolean getBooleanProperty(String name) throws Exception {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!hasProperty(name)) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted(name);
            logger.endMessage();
            throw new Exception("getBooleanProperty failed") ;
        }

        SettingsValue value = getProperty(name) ;
        if (!value.isBoolean()) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" property ").addQuoted(name).add(" is not a boolean");
            logger.endMessage();   
            throw new Exception("getBooleanProperty failed") ;         
        }

        return value.getBoolean() ;
    }    

    protected double getDoubleProperty(String name) throws Exception {
        double ret = Double.NaN ;
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!hasProperty(name)) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted(name);
            logger.endMessage();
            throw new Exception("getDoubleProperty failed") ;
        }

        SettingsValue value = getProperty(name) ;
        if (!value.isDouble() && !value.isInteger()) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" property ").addQuoted(name).add(" is not an integer");
            logger.endMessage();   
            throw new Exception("getDoubleProperty failed") ;         
        }
        else if (value.isDouble()) {
            ret = value.getDouble() ;
        }
        else if (value.isInteger()) {
            ret = value.getInteger() ;
        }

        return ret ;
    }

    protected double getDoublePropertyWithDefault(final String name, SettingsValue v, double ret) {

        try {
            if (v == null)
                v = getProperty(name) ;
            ret = v.getDouble();
        } catch (final BadParameterTypeException e) {
            final MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" event name ").addQuoted(name);
            logger.add(" value is not a double").endMessage();
        }

        return ret ;
    }    

    protected String getStringProperty(String name) throws Exception {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!hasProperty(name)) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted(name);
            logger.endMessage();
            throw new Exception("getStringProperty failed") ;
        }

        SettingsValue value = getProperty(name) ;
        if (!value.isString()) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" property ").addQuoted(name).add(" is not a string");
            logger.endMessage();   
            throw new Exception("getStringProperty failed") ;         
        }

        return value.getString() ;
    }
}