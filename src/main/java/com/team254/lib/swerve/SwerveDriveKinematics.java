// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.lib.swerve;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import java.util.Arrays;
import java.util.Collections;

import com.team254.lib.util.Util;
import org.ejml.simple.SimpleMatrix;

/**
 * Helper class that converts a chassis velocity (dx, dy, and dtheta components) into individual
 * module states (speed and angle).
 *
 * <p>The inverse kinematics (converting from a desired chassis velocity to individual module
 * states) uses the relative locations of the modules with respect to the center of rotation. The
 * center of rotation for inverse kinematics is also variable. This means that you can set your set
 * your center of rotation in a corner of the robot to perform special evasion maneuvers.
 *
 * <p>Forward kinematics (converting an array of module states into the overall chassis motion) is
 * performs the exact opposite of what inverse kinematics does. Since this is an overdetermined
 * system (more equations than variables), we use a least-squares approximation.
 *
 * <p>The inverse kinematics: [moduleStates] = [moduleLocations] * [chassisSpeeds] We take the
 * Moore-Penrose pseudoinverse of [moduleLocations] and then multiply by [moduleStates] to get our
 * chassis speeds.
 *
 * <p>Forward kinematics is also used for odometry -- determining the position of the robot on the
 * field using encoders and a gyro.
 */
public class SwerveDriveKinematics {
    private final SimpleMatrix m_inverseKinematics;
    private final SimpleMatrix m_forwardKinematics;

    private final int m_numModules;
    private final Translation2d[] m_modules;
    private Translation2d m_prevCoR = new Translation2d();
    private final Rotation2d[] m_rotations;

    /**
     * Constructs a swerve drive kinematics object. This takes in a variable number of wheel locations
     * as Translation2ds. The order in which you pass in the wheel locations is the same order that
     * you will receive the module states when performing inverse kinematics. It is also expected that
     * you pass in the module states in the same order when calling the forward kinematics methods.
     *
     * @param wheelsMeters The locations of the wheels relative to the physical center of the robot.
     */
    public SwerveDriveKinematics(Translation2d... wheelsMeters) {
        if (wheelsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }
        m_numModules = wheelsMeters.length;
        m_modules = Arrays.copyOf(wheelsMeters, m_numModules);
        m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);
        m_rotations = new Rotation2d[m_numModules];

        for (int i = 0; i < m_numModules; i++) {
            m_inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].y());
            m_inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +m_modules[i].x());
            m_rotations[i] = new Rotation2d(m_modules[i].x(), m_modules[i].y(), true);
        }
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();
    }

    /**
     * Performs inverse kinematics to return the module states from a desired chassis velocity. This
     * method is often used to convert joystick values into module speeds and angles.
     *
     * <p>This function also supports variable centers of rotation. During normal operations, the
     * center of rotation is usually the same as the physical center of the robot; therefore, the
     * argument is defaulted to that use case. However, if you wish to change the center of rotation
     * for evasive maneuvers, vision alignment, or for any other use case, you can do so.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
     *     rotation at one corner of the robot and provide a chassis speed that only has a dtheta
     *     component, the robot will rotate around that corner.
     * @return An array containing the module states. Use caution because these module states are not
     *     normalized. Sometimes, a user input may cause one of the module speeds to go above the
     *     attainable max velocity. Use the {@link #desaturateWheelSpeeds(SwerveModuleState[], double)
     *     DesaturateWheelSpeeds} function to rectify this issue.
     */
    @SuppressWarnings("LocalVariableName")
    public SwerveModuleState[] toSwerveModuleStates(
            ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
        if (!centerOfRotationMeters.equals(m_prevCoR)) {
            for (int i = 0; i < m_numModules; i++) {
                m_inverseKinematics.setRow(
                        i * 2 + 0,
                        0, /* Start Data */
                        1,
                        0,
                        -m_modules[i].y() + centerOfRotationMeters.y());
                m_inverseKinematics.setRow(
                        i * 2 + 1,
                        0, /* Start Data */
                        0,
                        1,
                        +m_modules[i].x() - centerOfRotationMeters.x());
            }
            m_prevCoR = centerOfRotationMeters;
        }

        var chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.setColumn(
                0,
                0,
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);

        var moduleStatesMatrix = m_inverseKinematics.mult(chassisSpeedsVector);
        SwerveModuleState[] moduleStates = new SwerveModuleState[m_numModules];

        for (int i = 0; i < m_numModules; i++) {
            double x = moduleStatesMatrix.get(i * 2, 0);
            double y = moduleStatesMatrix.get(i * 2 + 1, 0);

            double speed = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y, true);

            moduleStates[i] = new SwerveModuleState(speed, angle);
        }

        return moduleStates;
    }

    /**
     * Performs inverse kinematics. See {@link #toSwerveModuleStates(ChassisSpeeds, Translation2d)}
     * toSwerveModuleStates for more information.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @return An array containing the module states.
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        return toSwerveModuleStates(chassisSpeeds, new Translation2d());
    }

    /**
     * Performs forward kinematics to return the resulting chassis state from the given module states.
     * This method is often used for odometry -- determining the robot's position on the field using
     * data from the real-world speed and angle of each module on the robot.
     *
     * @param wheelStates The state of the modules (as a SwerveModuleState type) as measured from
     *     respective encoders and gyros. The order of the swerve module states should be same as
     *     passed into the constructor of this class.
     * @return The resulting chassis speed.
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... wheelStates) {
        if (wheelStates.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
        var moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            var module = wheelStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.cos());
            moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.sin());
        }

        var chassisSpeedsVector = m_forwardKinematics.mult(moduleStatesMatrix);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }

    public ChassisSpeeds toChassisSpeedWheelConstraints(SwerveModuleState... wheelStates) {
        if (wheelStates.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
        var constraintsMatrix = new SimpleMatrix(m_numModules * 2, 3);
        for (int i = 0; i < m_numModules; i++) {
            var module = wheelStates[i];

            var beta =
                    module.angle.rotateBy(
                            m_rotations[i].inverse()).rotateBy(Rotation2d.fromRadians(Math.PI / 2.0));

            //System.out.println(module);
            constraintsMatrix.setRow(i*2, 0,
                    module.angle.cos(),
                    module.angle.sin(),
                    -m_modules[i].norm()*beta.cos());
            constraintsMatrix.setRow(i*2 + 1, 0,
                    -module.angle.sin(),
                    module.angle.cos(),
                    m_modules[i].norm()*beta.sin());
        }
        //System.out.println(constraintsMatrix);

        var psuedoInv = constraintsMatrix.pseudoInverse();

        var enforcedConstraints = new SimpleMatrix(m_numModules*2, 1);
        for (int i = 0; i < m_numModules; i++) {
            enforcedConstraints.setRow(i*2, 0, wheelStates[i].speedMetersPerSecond);
            enforcedConstraints.setRow(i*2 + 1, 0, 0);
        }
        //System.out.println(enforcedConstraints);

        var chassisSpeedsVector = psuedoInv.mult(enforcedConstraints);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates Reference to array of module states. The array will be mutated with the
     *     normalized speeds!
     * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a module can reach.
     */
    public static void desaturateWheelSpeeds(
            SwerveModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
        double realMaxSpeed = Collections.max(Arrays.asList(moduleStates)).speedMetersPerSecond;
        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            for (SwerveModuleState moduleState : moduleStates) {
                moduleState.speedMetersPerSecond =
                        moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
            }
        }
    }

    public final Translation2d[] getModuleLocations() {
        return m_modules;
    }

    public int getNumModules() {
        return m_numModules;
    }

    public static ChassisSpeeds compensateForSkew(ChassisSpeeds speeds) {
        if (Util.epsilonEquals(speeds.omegaRadiansPerSecond,0, 1e-2)) return speeds;

        double kFactor = 0.1;
        double magnitude =
                Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) * speeds.omegaRadiansPerSecond * kFactor;

        Translation2d t = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(
                Rotation2d.fromDegrees(Math.signum(-speeds.omegaRadiansPerSecond) * 90.0));

        // t is now right direction, set right magnitude.
        t.scale(magnitude / t.norm());

        ChassisSpeeds r = new ChassisSpeeds(speeds.vxMetersPerSecond + t.x(),
                speeds.vyMetersPerSecond + t.y(),
                speeds.omegaRadiansPerSecond);
        return r;
    }
}

