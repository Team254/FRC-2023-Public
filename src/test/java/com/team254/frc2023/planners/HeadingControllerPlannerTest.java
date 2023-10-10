package com.team254.frc2023.planners;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.util.Util;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class HeadingControllerPlannerTest {

    @Test
    public void testDontOvershoot() {
        final boolean kPrint = false;
        HeadingControlPlanner planner = new HeadingControlPlanner();

        ChassisSpeeds speeds = new ChassisSpeeds(-1, 0, 0);
        Rotation2d rotation = Rotation2d.fromDegrees(-90);
        planner.setYawGoal(rotation);
        double commanded_rad_s = 0;
        double dt = 0.1;
        for (double ts = 0; ts < 5.0; ts += dt) {
            if (Util.epsilonEquals(ts, 2.0)) {
                planner.setYawGoal(Rotation2d.kPi);
            }
            Rotation2d delta_rot = Rotation2d.fromRadians(commanded_rad_s * 1.25 * dt);
            rotation = rotation.rotateBy(delta_rot);
            Twist2d twist = new Twist2d(speeds.vxMetersPerSecond * dt, speeds.vxMetersPerSecond * dt, delta_rot.getRadians());
            ChassisSpeeds output = planner.update(ts, speeds, twist, rotation);
            commanded_rad_s = output.omegaRadiansPerSecond;

            // Assert we dont overshoot the goal
            if (rotation.getRadians() > 0) {
                System.out.println(rotation.getRadians());
                Assertions.assertTrue(rotation.getRadians() > Math.PI - 0.3);
            }

            if (kPrint) {
                List<Double> nums = Arrays.asList(ts, commanded_rad_s, delta_rot.getRadians(), rotation.getRadians(), output.omegaRadiansPerSecond);
                List<String> strings = nums.stream().map(n -> String.format("%.03f", n))
                        .collect(Collectors.toList());
                System.out.println(String.join("\t", strings));
            }
        }

    }
}
