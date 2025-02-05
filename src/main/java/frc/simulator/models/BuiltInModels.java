package frc.simulator.models;

import frc.simulator.engine.ModelFactory;

public class BuiltInModels {
    private BuiltInModels() {
    }

    static public void registerBuiltinModels(ModelFactory factory) {
        factory.registerModel("fms", "frc.simulator.models.FMSModel");
        factory.registerModel("drivergamepad", "frc.simulator.models.DriverGamepadModel");
        factory.registerModel("swervedrive", "frc.simulator.models.SwerveDriveModel");
        factory.registerModel("oi2025", "frc.simulator.models.OI2025");
        factory.registerModel("dio", "frc.simulator.models.DigitalIOModel");  
    }
}
