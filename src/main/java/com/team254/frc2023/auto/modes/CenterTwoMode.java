package com.team254.frc2023.auto.modes;

import com.team254.frc2023.AutoModeSelector;
import com.team254.frc2023.auto.actions.*;
import com.team254.frc2023.subsystems.Drive;
import com.team254.frc2023.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class CenterTwoMode extends AutoModeBase {

    AutoModeSelector.DockMode mDockMode;
    public CenterTwoMode(AutoModeSelector.DockMode dockMode) {
        mDockMode = dockMode;
        addAction(new ParallelAction(
                new OrientModulesAction(0.8, Rotation2d.fromDegrees(0)),
                new SeriesAction(
                        new SuperstructureAction(Superstructure.GoalState.HIGH_SCORING_CONE),
                        new SuperstructureAction(Superstructure.GoalState.SCORE)
                )
        ));

        addAction(new ParallelAction(
                new SuperstructureAction(Superstructure.GoalState.STOW),
                new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().coopLinkStartToCoopFirstPickup, false),
                new SeriesAction(
                        new WaitForRegionAction(
                                new Translation2d(3.0, -1.27),
                                new Translation2d(6.0, 1.27)
                        ),
                        new SuperstructureAction(Superstructure.GoalState.PICKUP_LOW_CUBE)
                )
        ));

        addAction(new ParallelAction(
                new SuperstructureAction(Superstructure.GoalState.STOW),
                new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().coopFirstPickupToCoopSecondScore, false),
                new SeriesAction(
                        new WaitForRegionAction(
                                new Translation2d(0.0, -1.27),
                                new Translation2d(1.25, 1.27)
                        ),
                        new EnableLimelightAction(),
                        new SuperstructureAction(Superstructure.GoalState.HIGH_SCORING_CUBE)
                )
        ));

        if (mDockMode == AutoModeSelector.DockMode.DOCK) {
            DockActionNew dockAction = new DockActionNew(DockActionNew.StartingSide.NEAR, DockActionNew.ApproachOrientation.BACK_FIRST, 1.0, 15);
            addAction(new ParallelAction(
                    new DisableLimelightAction(),
                    new SeriesAction(
                            new SuperstructureAction(Superstructure.GoalState.SCORE),
                            new SuperstructureAction(Superstructure.GoalState.STOW)
                    ),
                    new InterruptableAction(new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().coopSecondScoreToDock), dockAction::getInclinationAboveThresholdForDock))
            );

            addAction(dockAction);
        } else {
            addAction(
                    new SeriesAction(
                            new SuperstructureAction(Superstructure.GoalState.SCORE),
                            new SuperstructureAction(Superstructure.GoalState.STOW)
                    )
            );
        }
    }
}
