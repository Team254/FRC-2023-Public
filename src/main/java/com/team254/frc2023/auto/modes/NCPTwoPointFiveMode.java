package com.team254.frc2023.auto.modes;

import com.team254.frc2023.AutoModeSelector;
import com.team254.frc2023.auto.AutoModeEndedException;
import com.team254.frc2023.auto.actions.*;
import com.team254.frc2023.subsystems.Drive;
import com.team254.frc2023.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class NCPTwoPointFiveMode extends AutoModeBase {

    AutoModeSelector.DockMode mDockMode;
    AutoModeSelector.PoopMode mPoopMode;
    public NCPTwoPointFiveMode(AutoModeSelector.DockMode dockMode, AutoModeSelector.PoopMode poopMode) {
        mDockMode = dockMode;
        mPoopMode = poopMode;
        addAction(new ParallelAction(
                new OrientModulesAction(0.0, Rotation2d.fromDegrees(0)),
                new SeriesAction(
                        new SuperstructureAction(Superstructure.GoalState.HIGH_SCORING_CONE_AUTO),
                        new SuperstructureAction(Superstructure.GoalState.SCORE)
                )
        ));

        Translation2d scoreRegionBL = new Translation2d(-1.0, -1.0);
        Translation2d scoreRegionTR = new Translation2d(0.15, 1.0);
        addAction(new ParallelAction(
                new SuperstructureAction(Superstructure.GoalState.STOW),
                new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().ncpScoringToCubePickup, false),
                new SeriesAction(
                        new WaitAction(1.5),
                        new SuperstructureAction(Superstructure.GoalState.PICKUP_LOW_CUBE),
                        new WaitAction(1.1),
                        new EnableLimelightAction(),
                        new SuperstructureAction(Superstructure.GoalState.STOW),
                        new WaitAction(1.0),
                        new SuperstructureAction(Superstructure.GoalState.PRE_HIGH_SCORING_CUBE, 0.0),
                        new WaitForRegionAction(scoreRegionBL, scoreRegionTR),
                        new SuperstructureAction(Superstructure.GoalState.HIGH_SCORING_CUBE),
                        new DisableLimelightAction()
                )
        ));

        if (mDockMode == AutoModeSelector.DockMode.DOCK) {
            DockActionNew dockAction = new DockActionNew(DockActionNew.StartingSide.NEAR, DockActionNew.ApproachOrientation.BACK_FIRST, 1.15, 15);
            addAction(new ParallelAction(
                    new SeriesAction(
                            new SuperstructureAction(Superstructure.GoalState.SCORE),
                            new SuperstructureAction(Superstructure.GoalState.STOW),
                            new WaitAction(0.35),
                            new SuperstructureAction(Superstructure.GoalState.PICKUP_LOW_CUBE)
                    ),
                    new InterruptableAction(new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().redNCPCubeScoringPositionToFarSideDockWithPickup), dockAction::getInclinationAboveThresholdForDock))
            );

            addAction(new DockActionNew(DockActionNew.StartingSide.FAR, DockActionNew.ApproachOrientation.FRONT_FIRST, 1.15, 15));


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