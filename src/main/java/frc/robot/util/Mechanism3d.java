package frc.robot.util;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mechanism3d extends SubsystemBase {

    private static final Distance outerElevatorCatch = Centimeters.of(75);

    private static final Transform3d armToAlgae = new Transform3d(
        Centimeters.of(-25),
        Centimeters.zero(),
        Centimeters.of(-10),
        new Rotation3d()
    );

    private static final Transform3d armToCoral = new Transform3d(
        Centimeters.of(-10),
        Centimeters.of(0.3),
        Centimeters.of(20.5),
        new Rotation3d()
    );

    private final String key_;

    private final Supplier<Pose2d> robotPoseSupplier_;
    private final Supplier<Distance> elevatorHeightSupplier_;
    private final Supplier<Angle> armAngleSupplier_;
    private final Supplier<Angle> climberAngleSupplier_;
    private final Supplier<Boolean> hasAlgaeSupplier_;
    private final Supplier<Boolean> hasCoralSupplier_;

    private Pose2d robotPose_ = new Pose2d();
    private Distance elevatorHeight_ = Meters.zero();
    private Angle armAngle_ = Degrees.zero();
    private Angle climberAngle_ = Degrees.zero();

    public Mechanism3d(
        String key,
        Supplier<Pose2d> robotPose,
        Supplier<Distance> elevatorHeight,
        Supplier<Angle> armAngle,
        Supplier<Angle> climberAngle,
        Supplier<Boolean> hasAlgae,
        Supplier<Boolean> hasCoral
    ) {
        key_ = key;
        robotPoseSupplier_ = robotPose;
        elevatorHeightSupplier_ = elevatorHeight;
        armAngleSupplier_ = armAngle;
        climberAngleSupplier_ = climberAngle;
        hasAlgaeSupplier_ = hasAlgae;
        hasCoralSupplier_ = hasCoral;
    }

    @Override
    public void periodic() {
        updateData();
        
        Pose3d outerElevator = elevatorHeight_.gt(outerElevatorCatch) ?
            new Pose3d(Meters.zero(), Meters.zero(), elevatorHeight_.minus(outerElevatorCatch), new Rotation3d()) :
            new Pose3d();

        Pose3d innerElevator = new Pose3d(Meters.zero(), Meters.zero(), elevatorHeight_, new Rotation3d());

        Pose3d arm = new Pose3d(
            Meters.of(0.28575),
            Meters.zero(),
            Meters.of(0.42).plus(elevatorHeight_),
            new Rotation3d(Degrees.zero(), armAngle_, Degrees.zero())
        );

        Pose3d climber = new Pose3d(
            Meters.of(-0.124),
            Meters.of(0.298),
            Meters.of(0.450),
            new Rotation3d(climberAngle_.unaryMinus(), Degrees.zero(), Degrees.zero())
        );

        Logger.recordOutput("Gamepiece3d/Algae/" + key_, hasAlgaeSupplier_.get() ?
            new Pose3d[] { robotToField(arm.plus(armToAlgae)) } : new Pose3d[0]
        );

        Logger.recordOutput("Gamepiece3d/Coral/" + key_, hasCoralSupplier_.get() ?
            new Pose3d[] { robotToField(arm.transformBy(armToCoral)) } : new Pose3d[0]
        );
        
        Pose3d[] result = { outerElevator, innerElevator, arm, climber };
        Logger.recordOutput("Mechanism3d/" + key_, result);
    }

    private Pose3d robotToField(Pose3d robot) {
        return new Pose3d(robotPose_).plus(new Transform3d(new Pose3d(), robot));
    }

    private void updateData() {
        robotPose_ = robotPoseSupplier_.get();
        if (elevatorHeightSupplier_.get() != null) elevatorHeight_ = elevatorHeightSupplier_.get();
        if (armAngleSupplier_.get() != null) armAngle_ = armAngleSupplier_.get();
        if (climberAngleSupplier_.get() != null) climberAngle_ = climberAngleSupplier_.get();
    }

}
