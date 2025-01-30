package frc.simulator.models;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import frc.simulator.utils.BadParameterTypeException;
import frc.simulator.utils.MessageLogger;
import frc.simulator.utils.MessageType;
import frc.simulator.utils.SettingsValue;
import frc.simulator.engine.SimulationEngine;
import frc.simulator.engine.SimulationModel;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public abstract class OIBaseModel extends SimulationModel {
    private static final String buttonEvent = "button";
    private static final String axisEvent = "axis";
    private static final String povEvent = "pov";

    private class MultiButtonConfig
    {
        public final String name ;
        public final int[] IOs;
        public final String[] states ;

        public MultiButtonConfig(String n, int[] ios, String[] st) {
            name = n ;
            IOs = ios ;
            this.states = st ;
        }
    }

    private Map<String, Integer> button_map_ ;
    private Map<String, Integer> led_map_ ;
    private List<MultiButtonConfig> multi_buttons_ ;

    private int index_ ;
    private int buttons_ ;
    private int button_count_ ;
    private float[] axes_ ;
    private short[] povs_ ;

    public OIBaseModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        button_map_ = new HashMap<String, Integer>() ;
        led_map_ = new HashMap<String, Integer>() ;
        multi_buttons_ = new ArrayList<>() ;
    }

    protected void setButtonMap(Map<String, Integer> map) {
        button_map_ = map ;
    }

    protected void setLedMap(Map<String, Integer> map) {
        led_map_ = map ;
    }

    protected boolean registerMultiButton(String name, int[] ios, String[] states) {
        multi_buttons_.add(new MultiButtonConfig(name, ios, states)) ;
        return true ;
    }

    public boolean create(SimulationEngine engine) {
        int count = 0;

        if (hasProperty("index")) {
            SettingsValue v = getProperty("index");
            if (!v.isInteger()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" has property ").addQuoted("index");
                logger.add(" but its not an integer");
                logger.endMessage();
                return false;
            }
            try {
                index_ = v.getInteger();
            } catch (BadParameterTypeException e) {
            }
        } else {
            MessageLogger logger = getEngine().getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted("index");
            logger.endMessage();
            return false;
        }

        if (hasProperty("axes")) {
            SettingsValue v = getProperty("axes");
            if (!v.isInteger()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" has property ").addQuoted("axes");
                logger.add(" but its not an integer");
                logger.endMessage();
                return false;
            }

            try {
                count = v.getInteger();
            } catch (BadParameterTypeException e) {
            }

            axes_ = new float[count];

            for (int i = 0; i < count; i++)
                axes_[i] = 0.0f;

            DriverStationDataJNI.setJoystickAxes((byte) index_, axes_);
        }

        if (hasProperty("buttons")) {
            SettingsValue v = getProperty("buttons");
            if (!v.isInteger()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" has property ").addQuoted("buttons");
                logger.add(" but its not an integer");
                logger.endMessage();
                return false;
            }

            try {
                button_count_ = v.getInteger();
            } catch (BadParameterTypeException e) {
            }

            buttons_ = 0;
            DriverStationDataJNI.setJoystickButtons((byte) index_, buttons_, button_count_);
        }

        if (hasProperty("povs")) {
            SettingsValue v = getProperty("povs");
            if (!v.isInteger()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" has property ").addQuoted("povs");
                logger.add(" but its not an integer");
                logger.endMessage();
                return false;
            }

            try {
                count = v.getInteger();
            } catch (BadParameterTypeException e) {
            }

            povs_ = new short[count];

            for (int i = 0; i < count; i++)
                povs_[i] = 0;

            DriverStationDataJNI.setJoystickPOVs((byte) index_, povs_);
        }

        DriverStationSim.notifyNewData();
        setCreated();
        return true ;
    }

    private int getValueIndex(MultiButtonConfig cfg, String v) {
        for(int i = 0 ; i < cfg.states.length ; i++) {
            if (cfg.states[i].equals(v))
                return i ;
        }

        return -1 ;
    }

    private MultiButtonConfig findButton(String name) {
        for(MultiButtonConfig button : multi_buttons_) {
            if (button.name.equals(name))
                return button ;
        }

        return null ;
    }

    private void processMultiButton(MultiButtonConfig button, SettingsValue value) {
        //
        // The value is required to be a string
        //
        if (!value.isString()) {
            MessageLogger logger = getEngine().getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" event name ").addQuoted(button.name);
            logger.add(" value is not a string").endMessage();
        }
        else {
            int entry ;
            
            try {
                entry = getValueIndex(button, value.getString()) ;
                if (entry == -1) {
                    MessageLogger logger = getEngine().getMessageLogger();
                    logger.startMessage(MessageType.Error);
                    logger.add("event: model ").addQuoted(getModelName());
                    logger.add(" instance ").addQuoted(getInstanceName());
                    logger.add(" event name ").addQuoted(button.name);
                    logger.add(" value is not valid, expected ") ;
                    for(int i = 0 ; i < button.states.length ; i++) {
                        if (i != 0)
                            logger.add(", ") ;
                        logger.addQuoted(button.states[i]) ;
                    }
                    logger.endMessage();
                }               
            }
            catch(BadParameterTypeException e) {
                entry = -1 ;
            }

            if (entry != -1) {
                for(int i = 0 ; i < button.IOs.length ; i++) {
                    int mask = (1 << i) ;
                    if ((entry & mask) == mask) {
                        buttons_ |= (1 << (button.IOs[i] - 1)) ;
                    }
                    else {
                        buttons_ &= ~(1 << (button.IOs[i] - 1)) ;
                    }
                }
            }
            DriverStationDataJNI.setJoystickButtons((byte) index_, buttons_, button_count_);
        }
    }

    private void processSimpleButtonByName(String name, int which, SettingsValue value) {
        if (!value.isBoolean()) {
            MessageLogger logger = getEngine().getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" event name ").addQuoted(name);
            logger.add(" value is not a boolean").endMessage();
        }
        else {
            try {
                if (value.getBoolean())
                    buttons_ |= (1 << (which - 1)) ;
                else
                    buttons_ &= ~(1 << (which - 1)) ;
            }
            catch(BadParameterTypeException e) {
            }

            DriverStationDataJNI.setJoystickButtons((byte) index_, buttons_, button_count_);
        }
    }

    public boolean processEvent(String name, SettingsValue value) {
        int which ;

        MultiButtonConfig button = findButton(name) ;
        if (button != null) {
            processMultiButton(button, value) ;
        }
        else if (button_map_.containsKey(name)) {
            processSimpleButtonByName(name, button_map_.get(name), value) ;

        }
        else if (name.startsWith(buttonEvent)) {
            try {
                which = Integer.parseInt(name.substring(buttonEvent.length())) ;
            }
            catch(NumberFormatException ex) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" is not valid, should be button followed by an integer (e.g. button2)").endMessage();    
                return true ;            
            }

            if (which > button_count_) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" exceeds the defined max button count").endMessage();
                return true ;
            }

            if (!value.isBoolean()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a boolean").endMessage();
                return true ;
            }

            try {
                if (value.getBoolean())
                    buttons_ |= (1 << (which - 1)) ;
                else
                    buttons_ &= ~(1 << (which - 1)) ;
            }
            catch(BadParameterTypeException e) {
            }

            DriverStationDataJNI.setJoystickButtons((byte) index_, buttons_, button_count_);
        }
        else if (name.startsWith(axisEvent)) {
            try {
                which = Integer.parseInt(name.substring(axisEvent.length())) ;
            }
            catch(NumberFormatException ex) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" is not valid, should be 'axis' followed by an integer (e.g. axis9)").endMessage();
                return true ;             
            }

            if (which > axes_.length) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" exceeds the defined max axis count").endMessage();
                return true ;
            }

            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();   
                return true ;             
            }            

            try {
                axes_[which] = (float)value.getDouble() ;
            }
            catch(BadParameterTypeException e) {
            }

            DriverStationDataJNI.setJoystickAxes((byte) index_, axes_) ;
        }
        else if (name.startsWith(povEvent)) {
            try {
                which = Integer.parseInt(name.substring(povEvent.length())) ;
            }
            catch(NumberFormatException ex) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" is not valid, should be 'pov' followed by an integer (e.g. pov0)").endMessage();   
                return true ;             
            }

            if (which > povs_.length) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" exceeds the defined max pov count").endMessage();
                return true ;
            }

            if (!value.isInteger()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not an integer").endMessage();   
                return true ;            
            }            

            try {
                povs_[which] = (short)value.getInteger() ;
            }
            catch(BadParameterTypeException e) {
            }

            DriverStationDataJNI.setJoystickPOVs((byte) index_, povs_) ;
        }        
        return true ;
    }

    @Override
    public void run(double dt) {
    }
}
