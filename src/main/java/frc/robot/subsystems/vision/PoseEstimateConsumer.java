
package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

@FunctionalInterface
public interface PoseEstimateConsumer {
    public void integrate(
        Pose2d robotPose,
        double timestampSecnds,
        Matrix<N3, N1> standardDeviations
    );

    public static PoseEstimateConsumer ignore() {
        return (Pose2d robotPose, double timestampSecnds, Matrix<N3, N1> standardDeviations) -> {};
    }
}