package com.team254.frc2023.auto.modes;

import com.team254.frc2023.AutoModeSelector;
import com.team254.frc2023.RobotState;
import com.team254.frc2023.auto.AutoModeEndedException;
import com.team254.frc2023.auto.actions.*;
import com.team254.frc2023.subsystems.Drive;
import com.team254.frc2023.subsystems.RobotStateEstimator;
import com.team254.frc2023.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

public class TestTrajectoryFollowingMode extends AutoModeBase {

    AutoModeSelector.DockMode mDockMode;
    public TestTrajectoryFollowingMode(AutoModeSelector.DockMode dockMode) {
        mDockMode = dockMode;
        addAction(new OrientModulesAction(1,Rotation2d.fromDegrees(0)));
        addAction(new SuperstructureAction(Superstructure.GoalState.UNSTOW));
        addAction(new SuperstructureAction(Superstructure.GoalState.STOW));
        for (int i = 0; i < 10; ++i) {
            addAction(new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().testTrajectory, false));
            //addAction(new EnableLimelightAction());
            addAction(new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().testTrajectory2, false));
            //addAction(new DisableLimelightAction());
        }

    }
}