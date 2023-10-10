package com.team254.frc2023.subsystems;

import com.team254.frc2023.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.opencv.core.Point;

public class LimelightTest {
    Limelight limelight = Limelight.getInstance();

    @Test
    public void testUndistortPoints() {
        long start = System.currentTimeMillis();
        double timeToRun = 5.0;//Float.POSITIVE_INFINITY;
        for (int i = 0; i <= Constants.kResolutionWidth; ++i) {
            for (int j = 0; j <= Constants.kResolutionHeight; ++j) {
                try{
                    //System.out.println("Doing (" + i + ", " + j + ").");
                    limelight.undistortFromOpenCV(new double[] {i / Constants.kResolutionWidth, j / Constants.kResolutionHeight});

                    long current = System.currentTimeMillis();

                    double sec = (current - start) / 1000F;
                    if (sec > timeToRun) {
                        return;
                    }
                } catch (Exception e) {
                    Assertions.fail("Failed for: (" + i + ", " + j + ").");
                }
            }
        }
    }
}
