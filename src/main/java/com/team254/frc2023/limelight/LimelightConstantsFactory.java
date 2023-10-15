package com.team254.frc2023.limelight;

import com.team254.frc2023.limelight.undistort.precomputedmaps.*;

import com.team254.frc2023.Constants;
import com.team254.frc2023.limelight.undistort.InterpolatingUndisortMap;
import com.team254.lib.geometry.Rotation2d;

public class LimelightConstantsFactory {
    public static LimelightConstants getConstantsForId(String id) {
        switch (id) {
            default: // Intentional fall through
            case "A":
                // Limelight used to procure target coordinates used in unit test
                return new LimelightConstants(
                        "A",
                        "Limelight A",
                        1.33296,
                        Rotation2d.fromDegrees(0.0),
                        Rotation2d.fromDegrees(-35.0),
                        new UndistortConstants(
                                new double[]{0.15545342, -0.46477633, 0.00277053, 0.0030637, 0.39464241},
                                new double[][]{{0.79862571, 0., 0.46119489},
                                        {0., 1.06276288, 0.48098825},
                                        {0., 0., 1.}}
                        ),
                        new InterpolatingUndisortMap((int)Constants.kResolutionWidth, (int)Constants.kResolutionHeight, new UndistortMap_Limelight_A_640x480())
                );
            case "B":
                // Limelight used to procure target coordinates used in unit test
                return new LimelightConstants(
                        "B",
                        "Limelight B",
                        // Measured from CAD: 2/25: 133296 cm
                        1.33296,
                        Rotation2d.fromDegrees(0),//-2.75),
                        // Measured from CAD: 40 degrees
                        Rotation2d.fromDegrees(-35.0),
                        new UndistortConstants(
                                new double[]{0.13077097, -0.43223309, 0.00216471, -0.00145679, 0.36181027},
                                new double[][]{{0.81755082, 0., 0.45041716},
                                        {0., 1.09093867, 0.52956206},
                                        {0., 0., 1.}}
                        ),
                        new InterpolatingUndisortMap((int)Constants.kResolutionWidth, (int)Constants.kResolutionHeight, new UndistortMap_Limelight_B_640x480())
                );

        }
    }

}
