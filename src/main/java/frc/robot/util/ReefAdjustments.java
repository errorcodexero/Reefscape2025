package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.units.measure.Distance;

public class ReefAdjustments {
    public final Map<Integer, ReefAdjustmentDistances> adjustments = new HashMap<Integer,ReefAdjustments.ReefAdjustmentDistances>() ;

    public static final ReefAdjustments AdjustmentData = new ReefAdjustments();

    public class ReefAdjustmentDistances {
        public Distance left_right_left_ ;                  // Left/right adjustment for the left side of the reef face
        public Distance left_right_right_ ;                 // Left/right adjustment for the right side of the reef face
        public Distance forward_back_left_ ;                // Forward/back adjustment for the left side of the reef face
        public Distance forward_back_right_ ;               // Forward/back adjustment for the right side of the reef face

        public ReefAdjustmentDistances(Distance left_right_left, Distance left_right_right, Distance forward_back_left, Distance forward_back_right) {
            left_right_left_ = left_right_left;
            left_right_right_ = left_right_right;
            forward_back_left_ = forward_back_left;
            forward_back_right_ = forward_back_right;
        }
    }

    private ReefAdjustments() {
        //
        // The faces are using the 1425 nomenclature
        //
        // A positive value on the front/back adjustment move the face further from the center of the reef
        // A positive value on the left/right adjustment moves the face to the right
        //
        adjustments.put(6, new ReefAdjustmentDistances(
            // April tag 6 - Red face 2
            Inches.of(0.0),             // Red face 2 - left side - left/right adjustment
            Inches.of(0.0),             // Red face 2 - right side - left/right adjustment
            Inches.of(0.0),             // Red face 2 - left side - front/back adjustment
            Inches.of(0.0)));           // Red face 2 - right side - front/back adjustment

        adjustments.put(7, new ReefAdjustmentDistances(
            // April tag 7 - Red face 1
            Inches.of(0.0),             // Red face 1 - left side - left/right adjustment
            Inches.of(0.0),             // Red face 1 - right side - left/right adjustment
            Inches.of(0.0),             // Red face 1 - left side - front/back adjustment
            Inches.of(0.0)));           // Red face 1 - right side - front/back adjustment

        adjustments.put(8, new ReefAdjustmentDistances(
            // April tag 8 - Red face 6
            Inches.of(0.0),             // Red face 6 - left side - left/right adjustment
            Inches.of(0.0),             // Red face 6 - right side - left/right adjustment
            Inches.of(0.0),             // Red face 6 - left side - front/back adjustment
            Inches.of(0.0)));           // Red face 6 - right side - front/back adjustment

        adjustments.put(9, new ReefAdjustmentDistances(
            // April tag 9 - Red face 5
            Inches.of(0.0),             // Red face 5 - left side - left/right adjustment
            Inches.of(0.0),             // Red face 5 - right side - left/right adjustment
            Inches.of(0.0),             // Red face 5 - left side - front/back adjustment
            Inches.of(0.0)));           // Red face 5 - right side - front/back adjustment

        adjustments.put(10, new ReefAdjustmentDistances(
            // April tag 10 - Red face 4
            Inches.of(0.0),             // Red face 4 - left side - left/right adjustment
            Inches.of(0.0),             // Red face 4 - right side - left/right adjustment
            Inches.of(0.0),             // Red face 4 - left side - front/back adjustment
            Inches.of(0.0)));           // Red face 4 - right side - front/back adjustment

        adjustments.put(11, new ReefAdjustmentDistances(
            // April tag 11 - Red face 3
            Inches.of(0.0),             // Red face 3 - left side - left/right adjustment
            Inches.of(0.0),             // Red face 3 - right side - left/right adjustment
            Inches.of(0.0),             // Red face 3 - left side - front/back adjustment
            Inches.of(0.0)));           // Red face 3 - right side - front/back adjustment

        adjustments.put(17, new ReefAdjustmentDistances(
            // April tag 17 - Blue face 6
            Inches.of(0.0),             // Blue face 6 - left side - left/right adjustment 
            Inches.of(0.0),             // Blue face 6 - right side - left/right adjustment
            Inches.of(0.0),             // Blue face 6 - left side - front/back adjustment
            Inches.of(0.0)));           // Blue face 6 - right side - front/back adjustment

        adjustments.put(18, new ReefAdjustmentDistances(
            // April tag 18 - Blue face 1
            Inches.of(0.0),             // Blue face 1 - left side - left/right adjustment
            Inches.of(0.0),             // Blue face 1 - right side - left/right adjustment
            Inches.of(0.0),             // Blue face 1 - left side - front/back adjustment
            Inches.of(0.0)));           // Blue face 1 - right side - front/back adjustment

        adjustments.put(19, new ReefAdjustmentDistances(
            // April tag 19 - Blue face 2
            Inches.of(0.0),             // Blue face 2 - left side - left/right adjustment
            Inches.of(0.0),             // Blue face 2 - right side - left/right adjustment
            Inches.of(0.0),             // Blue face 2 - left side - front/back adjustment
            Inches.of(0.0)));           // Blue face 2 - right side - front/back adjustment

        adjustments.put(20, new ReefAdjustmentDistances(
            // April tag 20 - Blue face 3
            Inches.of(0.0),             // Blue face 3 - left side - left/right adjustment
            Inches.of(0.0),             // Blue face 3 - right side - left/right adjustment
            Inches.of(0.0),             // Blue face 3 - left side - front/back adjustment
            Inches.of(0.0)));           // Blue face 3 - right side - front/back adjustment

        adjustments.put(21, new ReefAdjustmentDistances(
            // April tag 21 - Blue face 4
            Inches.of(0.0),             // Blue face 4 - left side - left/right adjustment
            Inches.of(0.0),             // Blue face 4 - right side - left/right adjustment
            Inches.of(0.0),             // Blue face 4 - left side - front/back adjustment
            Inches.of(0.0)));           // Blue face 4 - right side - front/back adjustment

        adjustments.put(22, new ReefAdjustmentDistances(
            // April tag 22 - Blue face 5
            Inches.of(0.0),             // Blue face 5 - left side - left/right adjustment 
            Inches.of(0.0),             // Blue face 5 - right side - left/right adjustment
            Inches.of(0.0),             // Blue face 5 - left side - front/back adjustment
            Inches.of(0.0)));           // Blue face 5 - right side - front/back adjustment
    }
}
