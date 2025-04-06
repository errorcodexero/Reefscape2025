package frc.simulator.models;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.XboxController;

import frc.simulator.engine.SimulationModel;
import frc.simulator.engine.SimulationEngine;
import java.util.Map;

import org.xerosw.util.BadParameterTypeException;
import org.xerosw.util.MessageLogger;
import org.xerosw.util.MessageType;
import org.xerosw.util.SettingsValue;

public class DriverGamepadModel extends SimulationModel {
    private static final String buttonEvent = "button";
    private static final String axisEvent = "axis";
    private static final String povEvent = "pov";

    private static final Map<String, Integer> buttonMap = Map.of(
        "a", 1,
        "b", 2,
        "x", 3,
        "y", 4,
        "lb", 5,
        "rb", 6,
        "back", 7,
        "start", 8,
        "lstick", 9,
        "rstick", 10
    ) ;

    private int index_ ;
    private int buttons_ ;
    private int button_count_ ;
    private float[] axes_ ;
    private short[] povs_ ;    

    public DriverGamepadModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);
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
                povs_[i] = -1 ;
        }

        setCreated();
        
        DriverStationDataJNI.setJoystickButtons((byte) index_, buttons_, button_count_);        
        DriverStationDataJNI.setJoystickAxes((byte) index_, axes_) ;
        DriverStationDataJNI.setJoystickPOVs((byte) index_, povs_) ;

        return true ;
    }

    public void run(double dt) {
    }

    private void doOneButton(String name, int which, SettingsValue value) {
        if (which > button_count_) {
            MessageLogger logger = getEngine().getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" event name ").addQuoted(name);
            logger.add(" exceeds the defined max button count").endMessage();
            return ;
        }

        if (!value.isBoolean()) {
       
            return ;         
        }

        try {
            if (value.getBoolean())
                buttons_ |= (1 << (which - 1)) ;
            else
                buttons_ &= ~(1 << (which - 1)) ;
        }
        catch(BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" event name ").addQuoted(name);
            logger.add(" value is not a boolean").endMessage();            
        }
    }

    public boolean processEvent(String name, SettingsValue value) {
        int which = 0 ;

        if (buttonMap.containsKey(name)) {
            which = buttonMap.get(name) ;
            doOneButton(name, which, value) ;
        } else if (name.startsWith(buttonEvent)) {
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
            doOneButton(name, which, value) ;
        }
        else if (name.equals("ltrigger")) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();                
            }            

            try {
                axes_[XboxController.Axis.kLeftTrigger.value] = (float)value.getDouble() ;
            }
            catch(Exception ex) {
            }            
        }        
        else if (name.equals("rtrigger")) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();                
            }            

            try {
                axes_[XboxController.Axis.kRightTrigger.value] = (float)value.getDouble() ;
            }
            catch(Exception ex) {
            }            
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
            }            

            try {
                axes_[which] = (float)value.getDouble() ;
            }
            catch(BadParameterTypeException e) {
            }
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
            }            

            try {
                povs_[which] = (short)value.getInteger() ;
            }
            catch(BadParameterTypeException e) {
            }
        }        

        DriverStationDataJNI.setJoystickButtons((byte) index_, buttons_, button_count_);        
        DriverStationDataJNI.setJoystickAxes((byte) index_, axes_) ;
        DriverStationDataJNI.setJoystickPOVs((byte) index_, povs_) ;

        return true ;
    }
}