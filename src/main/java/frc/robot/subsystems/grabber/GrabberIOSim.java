package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotType;

public class GrabberIOSim extends GrabberIOHardware {

    private static final Distance collectTolerance = Meters.one();

    private final Supplier<Pose2d> robotPose_;
    private final List<Pose2d> stations_;

    // 1, 2, 13, 12
    
    public GrabberIOSim(Supplier<Pose2d> robotPose) throws Exception {
        robotPose_ = robotPose;

        stations_ = List.of(
            tagPose(1),
            tagPose(2),
            tagPose(12),
            tagPose(13)
        );
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        super.updateInputs(inputs);

        if (Constants.getRobot() != RobotType.XEROSIM) {
            Pose2d robot = robotPose_.get();
            Pose2d nearestStation = robot.nearest(stations_);

            inputs.coralSensor = robot.getTranslation().getDistance(nearestStation.getTranslation()) <= collectTolerance.in(Meters);
            inputs.algaeSensor1 = true;
            inputs.algaeSensor2 = true;
        }
    }

    private Pose2d tagPose(int id) {
        return FieldConstants.layout.getTagPose(id).orElseThrow().toPose2d();
    }
}