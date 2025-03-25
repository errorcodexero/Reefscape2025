package frc.simulator.models;

import java.util.HashMap;
import java.util.Map;

import frc.robot.subsystems.oi.OIConstants;
import frc.robot.subsystems.oi.OIConstants.OILed;
import frc.simulator.engine.SimulationEngine;

public class OI2025 extends OIBaseModel {
    private static final Map<String, Integer> buttonMap = new HashMap<String, Integer>() ;
    private static final Map<String, Integer> ledMap = new HashMap<String, Integer>() ;
   
    public OI2025(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;

        setButtonMap(getButtonMap());
        setLedMap(getLedMap());
    }

    private Map<String, Integer> getButtonMap() {
        buttonMap.put("eject", Integer.valueOf(OIConstants.Buttons.kEject)) ;
        buttonMap.put("abort", Integer.valueOf(OIConstants.Buttons.kAbort)) ;
        buttonMap.put("execute", Integer.valueOf(OIConstants.Buttons.kExecute)) ;
        buttonMap.put("l1", Integer.valueOf(OIConstants.Buttons.kCoralL1)) ;
        buttonMap.put("l2", Integer.valueOf(OIConstants.Buttons.kCoralL2)) ;
        buttonMap.put("l3", Integer.valueOf(OIConstants.Buttons.kCoralL3)) ;
        buttonMap.put("l4", Integer.valueOf(OIConstants.Buttons.kCoralL4)) ;
        buttonMap.put("coral-collect", Integer.valueOf(OIConstants.Buttons.kCoralCollect)) ;
        buttonMap.put("coral-place", Integer.valueOf(OIConstants.Buttons.kCoralPlace)) ;
        buttonMap.put("algae-score", Integer.valueOf(OIConstants.Buttons.kAlgaeScore)) ;
        buttonMap.put("algae-reef", Integer.valueOf(OIConstants.Buttons.kAlgaeReef)) ;
        buttonMap.put("climb-deploy", Integer.valueOf(OIConstants.Buttons.kClimbDeploy)) ;
        buttonMap.put("climb-execute", Integer.valueOf(OIConstants.Buttons.kClimbExecute)) ;
        buttonMap.put("climb-lock", Integer.valueOf(OIConstants.Buttons.kClimbLock)) ;
        buttonMap.put("coral-side", Integer.valueOf(OIConstants.Buttons.kCoralSide)) ;

        return buttonMap ;
    }

    private Map<String, Integer> getLedMap() {
        ledMap.put("coral-l1", OILed.CoralL1.value) ;
        ledMap.put("coral-l2", OILed.CoralL2.value) ;
        ledMap.put("coral-l3", OILed.CoralL3.value) ;
        ledMap.put("coral-l4", OILed.CoralL4.value) ;
        ledMap.put("coral-collect", OILed.CollectCoral.value) ;
        ledMap.put("coral-place", OILed.PlaceCoral.value) ;
        ledMap.put("algae-ground", OILed.CollectAlgaeReefEject.value) ;
        ledMap.put("algae-score", OILed.ScoreAlgae.value) ;
        ledMap.put("algae-reef", OILed.CollectAlgaeReefKeep.value) ;
        ledMap.put("coral-left", OILed.CoralLeft.value) ;
        ledMap.put("coral-right", OILed.CoralRight.value) ;
        return ledMap ;
    }
}
