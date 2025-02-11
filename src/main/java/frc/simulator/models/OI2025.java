package frc.simulator.models;

import java.util.HashMap;
import java.util.Map;

import frc.robot.subsystems.oi.OIConstants;
import frc.robot.subsystems.oi.OISubsystem;
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
        buttonMap.put("algae-ground", Integer.valueOf(OIConstants.Buttons.kAlgaeGround)) ;
        buttonMap.put("algae-score", Integer.valueOf(OIConstants.Buttons.kAlgaeScore)) ;
        buttonMap.put("algae-reef", Integer.valueOf(OIConstants.Buttons.kAlgaeReef)) ;
        buttonMap.put("climb-deploy", Integer.valueOf(OIConstants.Buttons.kClimbDeploy)) ;
        buttonMap.put("climb-execute", Integer.valueOf(OIConstants.Buttons.kClimbExecute)) ;
        buttonMap.put("climb-lock", Integer.valueOf(OIConstants.Buttons.kClimbLock)) ;
        buttonMap.put("coral-side", Integer.valueOf(OIConstants.Buttons.kCoralSide)) ;

        return buttonMap ;
    }

    private Map<String, Integer> getLedMap() {
        ledMap.put("coral-l1", OISubsystem.OILed.CoralL1.value) ;
        ledMap.put("coral-l2", OISubsystem.OILed.CoralL2.value) ;
        ledMap.put("coral-l3", OISubsystem.OILed.CoralL3.value) ;
        ledMap.put("coral-l4", OISubsystem.OILed.CoralL4.value) ;
        ledMap.put("coral-collect", OISubsystem.OILed.CollectCoral.value) ;
        ledMap.put("coral-place", OISubsystem.OILed.PlaceCoral.value) ;
        ledMap.put("algae-ground", OISubsystem.OILed.CollectAlgaeGround.value) ;
        ledMap.put("algae-score", OISubsystem.OILed.ScoreAlgae.value) ;
        ledMap.put("algae-reef", OISubsystem.OILed.CollectAlgaeReef.value) ;
        ledMap.put("climb-deploy", OISubsystem.OILed.ClimbDeploy.value) ;
        ledMap.put("climb-execute", OISubsystem.OILed.ClimbExecute.value) ;
        ledMap.put("coral-left", OISubsystem.OILed.CoralLeft.value) ;
        ledMap.put("coral-right", OISubsystem.OILed.CoralRight.value) ;
        return ledMap ;
    }
}
