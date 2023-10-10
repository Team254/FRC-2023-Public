package com.team254.frc2023.auto.modes;

import com.team254.frc2023.AutoModeSelector;
import com.team254.frc2023.auto.actions.*;
import com.team254.frc2023.subsystems.Drive;
import com.team254.frc2023.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class CenterOneMode extends AutoModeBase {

    AutoModeSelector.DockMode mDockMode;
    public CenterOneMode(AutoModeSelector.DockMode dockMode) {
        mDockMode = dockMode;
        addAction(new ParallelAction(
                new OrientModulesAction(0.8, Rotation2d.fromDegrees(0)),
                new SeriesAction(
                        new SuperstructureAction(Superstructure.GoalState.HIGH_SCORING_CONE),
                        new SuperstructureAction(Superstructure.GoalState.SCORE)
                )
        ));

        if (mDockMode == AutoModeSelector.DockMode.DOCK) {
            DockActionNew dockAction = new DockActionNew(DockActionNew.StartingSide.NEAR, DockActionNew.ApproachOrientation.BACK_FIRST, 1.0, 15);
            addAction(new ParallelAction(
                    new DisableLimelightAction(),
                    new SuperstructureAction(Superstructure.GoalState.STOW),
                    new InterruptableAction(new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().coopFirstScoreToDock), dockAction::getInclinationAboveThresholdForDock))
            );

            addAction(dockAction);
        } else {
            addAction(
                    new SuperstructureAction(Superstructure.GoalState.STOW)
            );
        }
    }
}
