package com.team254.frc2023.limelight.undistort;


import com.team254.frc2023.Constants;
import com.team254.frc2023.limelight.LimelightConstantsFactory;
import com.team254.frc2023.subsystems.Limelight;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class PrecomputedMapTest {

    private void doTest(UndistortMap map) throws Exception {
        // Iterate by some non-common divisor
        for (int x = 0 ; x < (int) Constants.kResolutionWidth;  x += 6) {
            for (int y = 0 ; y < (int) Constants.kResolutionHeight; y += 7) {
                double[] cvIn = new double[] { x / Constants.kResolutionWidth, y / Constants.kResolutionHeight };
                double[] fromOpenCv = Limelight.getInstance().undistortFromOpenCV(cvIn);
                double[] mapped = map.pixelToUndistortedNormalized(x, y);
                double epsilon = y < ((int)Constants.kResolutionHeight - 1) ? 0.001 : 0.002;
                Assertions.assertArrayEquals(mapped, fromOpenCv, epsilon);
            }
        }
    }
    @Test
    public void testPrecomputedApproximatesOpenCvCall() throws Exception {
        doTest(LimelightConstantsFactory.getConstantsForId(Constants.kCompLLId).getUndistortMap());
        doTest(LimelightConstantsFactory.getConstantsForId(Constants.kPracticeLLId).getUndistortMap());
    }
}
