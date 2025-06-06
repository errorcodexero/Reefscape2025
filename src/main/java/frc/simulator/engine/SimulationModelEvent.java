package frc.simulator.engine;

import org.xerosw.util.BadParameterTypeException;
import org.xerosw.util.MessageLogger;
import org.xerosw.util.MessageType;
import org.xerosw.util.SettingsValue;

public class SimulationModelEvent extends SimulationEvent {
    public SimulationModelEvent(double t, String model, String instance, String name, SettingsValue v) {
        super(t);

        model_ = model;
        instance_ = instance;
        name_ = name;
        value_ = v;
    }

    public String getModel() {
        return model_;
    }

    public String getInstance() {
        return instance_;
    }

    public String getName() {
        return name_;
    }

    public SettingsValue getValue() {
        return value_;
    }

    public String toString() {
        String str = "SimulationModelEvent:";
        str += " model=" + model_;
        str += " instance=" + instance_;
        str += " name=" + name_;
        str += " value=" + value_.toString();

        return str;
    }

    public void run(SimulationEngine engine) {
        if (name_.equals("comment")) {
            MessageLogger logger = engine.getMessageLogger();
            logger.startMessage(MessageType.Info);
            logger.add("simulation comment ");
            if (value_.isString()) {
                try {
                    logger.addQuoted(value_.getString());
                } catch (BadParameterTypeException e) {
                }
            }
            else {
                logger.addQuoted(value_.toString()) ;
            }
            logger.endMessage();
        }
        else {
            SimulationModel model = engine.getModelByNameInst(model_, instance_) ;
            if (model != null)
                model.processEvent(name_, value_) ;
            else {
                MessageLogger logger = engine.getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("cannot process event ").addQuoted(toString()) ;
                logger.add(" model ").addQuoted(model_).add(" instance ").addQuoted(instance_) ;
                logger.add(" does not exist") ;
            }
        }
    }

    private String model_ ;
    private String instance_ ;
    private String name_ ;
    private SettingsValue value_ ;
}