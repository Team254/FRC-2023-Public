package com.team254.frc2023.lib.spline;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import com.team254.lib.spline.*;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

public class QuinticHermiteOptimizerTest {
    private static double kEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        Pose2d a = new Pose2d(new Translation2d(0, 100), Rotation2d.fromDegrees(270));
        Pose2d b = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d c = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(90));

        List<QuinticHermitePoseSplineNonholonomic> splines = new ArrayList<>();
        splines.add(new QuinticHermitePoseSplineNonholonomic(a, b));
        splines.add(new QuinticHermitePoseSplineNonholonomic(b, c));

        long startTime = System.currentTimeMillis();
        Assertions.assertTrue(QuinticHermitePoseSplineNonholonomic.optimizeSpline(splines) < 0.014);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));

        Pose2d d = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90));
        Pose2d e = new Pose2d(new Translation2d(0, 50), Rotation2d.fromDegrees(0));
        Pose2d f = new Pose2d(new Translation2d(100, 0), Rotation2d.fromDegrees(90));
        Pose2d g = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(0));

        List<QuinticHermitePoseSplineNonholonomic> splines1 = new ArrayList<>();
        splines1.add(new QuinticHermitePoseSplineNonholonomic(d, e));
        splines1.add(new QuinticHermitePoseSplineNonholonomic(e, f));
        splines1.add(new QuinticHermitePoseSplineNonholonomic(f, g));

        startTime = System.currentTimeMillis();
        Assertions.assertTrue(QuinticHermitePoseSplineNonholonomic.optimizeSpline(splines1) < 0.16);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));


        Pose2d h = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
        Pose2d i = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d j = new Pose2d(new Translation2d(100, 50), Rotation2d.fromDegrees(45));
        Pose2d k = new Pose2d(new Translation2d(150, 0), Rotation2d.fromDegrees(270));
        Pose2d l = new Pose2d(new Translation2d(150, -50), Rotation2d.fromDegrees(270));

        List<QuinticHermitePoseSplineNonholonomic> splines2 = new ArrayList<>();
        splines2.add(new QuinticHermitePoseSplineNonholonomic(h, i));
        splines2.add(new QuinticHermitePoseSplineNonholonomic(i, j));
        splines2.add(new QuinticHermitePoseSplineNonholonomic(j, k));
        splines2.add(new QuinticHermitePoseSplineNonholonomic(k, l));

        startTime = System.currentTimeMillis();
        Assertions.assertTrue(QuinticHermitePoseSplineNonholonomic.optimizeSpline(splines2) < 0.05);
        Assertions.assertEquals(splines2.get(0).getCurvature(1.0), 0.0, kEpsilon);
        Assertions.assertEquals(splines2.get(2).getCurvature(1.0), 0.0, kEpsilon);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));
    }
}
