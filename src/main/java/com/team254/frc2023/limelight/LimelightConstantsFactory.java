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
//            case "C":
//                // Limelight used to procure target coordinates used in unit test
//                return new LimelightConstants(
//                        "C",
//                        "Limelight C",
//                        // Measured from CAD: 2/25: 133296 cm
//                        1.33296,
//                        Rotation2d.fromDegrees(0),//-2.75),
//                        // Measured from CAD: 40 degrees
//                        Rotation2d.fromDegrees(-35.0),
//                        new UndistortConstants(
//                                new double[]{0.2926471, -1.10146642, 0.01213953, -0.01274662, 1.51510422},
//                                new double[][]{{0.86359887, 0., 0.43713074},
//                                        {0., 1.08831551, 0.52837743},
//                                        {0., 0., 1.}}
//                        ),
//                        new InterpolatingUndisortMap((int)Constants.kResolutionWidth, (int)Constants.kResolutionHeight, new UndistortMap_Limelight_C_640x480())
//                );

//            case "D":
//                return new LimelightConstants(
//                        "D",
//                        "Limelight D",
//                        // Measured from CAD: 2/25: 133296 cm
//                        1.33296,
//                        Rotation2d.fromDegrees(0),//-2.75),
//                        // Measured from CAD: 40 degrees
//                        Rotation2d.fromDegrees(-35.0),
//                        new UndistortConstants(
//                                new double[]{3.00837416e-01, -1.16186603e+00, 1.04699386e-02, -1.48493384e-04, 1.48709128e+00},
//                                new double[][]{{0.84806274, 0., 0.48627109}, {0., 1.07533023, 0.51690732}, {0., 0., 1.}}
//                        ),
//                        new InterpolatingUndisortMap((int)Constants.kResolutionWidth, (int)Constants.kResolutionHeight, new UndistortMap_Limelight_D_640x480())
//                );

        }
    }

}
