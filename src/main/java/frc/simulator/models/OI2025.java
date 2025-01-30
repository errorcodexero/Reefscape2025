package frc.simulator.models;

import java.util.HashMap;
import java.util.Map;

import frc.robot.subsystems.oi.OIConstants;
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
        buttonMap.put("coral-l1", Integer.valueOf(OIConstants.Buttons.kCoralL1)) ;
        buttonMap.put("coral-l2", Integer.valueOf(OIConstants.Buttons.kCoralL2)) ;
        buttonMap.put("coral-l3", Integer.valueOf(OIConstants.Buttons.kCoralL3)) ;
        buttonMap.put("coral-l4", Integer.valueOf(OIConstants.Buttons.kCoralL4)) ;
        buttonMap.put("coral-collect", Integer.valueOf(OIConstants.Buttons.kCoralCollect)) ;
        buttonMap.put("coral-place", Integer.valueOf(OIConstants.Buttons.kCoralPlace)) ;
        buttonMap.put("algae-ground", Integer.valueOf(OIConstants.Buttons.kAlgaeGround)) ;
        buttonMap.put("algae-score", Integer.valueOf(OIConstants.Buttons.kAlgaeScore)) ;
        buttonMap.put("algae-collect-l2", Integer.valueOf(OIConstants.Buttons.kAlgaeCollectL2)) ;
        buttonMap.put("algae-collect-l3", Integer.valueOf(OIConstants.Buttons.kAlgaeCollectL3)) ;
        buttonMap.put("climb-deploy", Integer.valueOf(OIConstants.Buttons.kClimbDeploy)) ;
        buttonMap.put("climb-execute", Integer.valueOf(OIConstants.Buttons.kClimbExecute)) ;
        buttonMap.put("climb-lock", Integer.valueOf(OIConstants.Buttons.kClimbLock)) ;
        buttonMap.put("coral-side", Integer.valueOf(OIConstants.Buttons.kCoralSide)) ;

        return buttonMap ;
    }

    private Map<String, Integer> getLedMap() {
        ledMap.put("eject", Integer.valueOf(OIConstants.LEDs.kEject)) ;
        ledMap.put("coral-l1", Integer.valueOf(OIConstants.LEDs.kCoralL1)) ;
        ledMap.put("coral-l2", Integer.valueOf(OIConstants.LEDs.kCoralL2)) ;
        ledMap.put("coral-l3", Integer.valueOf(OIConstants.LEDs.kCoralL3)) ;
        ledMap.put("coral-l4", Integer.valueOf(OIConstants.LEDs.kCoralL4)) ;
        ledMap.put("coral-collect", Integer.valueOf(OIConstants.LEDs.kCoralCollect)) ;
        ledMap.put("coral-place", Integer.valueOf(OIConstants.LEDs.kCoralPlace)) ;
        ledMap.put("algae-ground", Integer.valueOf(OIConstants.LEDs.kAlgaeGround)) ;
        ledMap.put("algae-score", Integer.valueOf(OIConstants.LEDs.kAlgaeScore)) ;
        ledMap.put("algae-collect-l2", Integer.valueOf(OIConstants.LEDs.kAlgaeCollectL2)) ;
        ledMap.put("algae-collect-l3", Integer.valueOf(OIConstants.LEDs.kAlgaeCollectL3)) ;
        ledMap.put("climb-deploy", Integer.valueOf(OIConstants.LEDs.kClimbDeploy)) ;
        ledMap.put("climb-execute", Integer.valueOf(OIConstants.LEDs.kClimbExecute)) ;
        ledMap.put("coral-left", Integer.valueOf(OIConstants.LEDs.kCoralLeft)) ;
        ledMap.put("coral-right", Integer.valueOf(OIConstants.LEDs.kCoralRight)) ;

        return ledMap ;
    }
}
