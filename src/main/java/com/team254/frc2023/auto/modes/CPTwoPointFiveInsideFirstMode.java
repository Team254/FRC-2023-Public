package com.team254.frc2023.auto.modes;

import com.team254.frc2023.AutoModeSelector;
import com.team254.frc2023.auto.actions.*;
import com.team254.frc2023.subsystems.Drive;
import com.team254.frc2023.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
/*
 * Cable Protector/Bump Side - Scores High Cone, then picks up inside cube, scores it high, then picks up outside cube and balances on bridge.
 * */
public class CPTwoPointFiveInsideFirstMode extends AutoModeBase {

    AutoModeSelector.DockMode mDockMode;
    public CPTwoPointFiveInsideFirstMode(AutoModeSelector.DockMode dockMode) {
        mDockMode = dockMode;
        addAction(new ParallelAction(
                new OrientModulesAction(0.0, Rotation2d.fromDegrees(0)),
                new SeriesAction(
                        new SuperstructureAction(Superstructure.GoalState.HIGH_SCORING_CONE_AUTO),
                        new SuperstructureAction(Superstructure.GoalState.SCORE)
                )
        ));
        addAction(new SeriesAction(
                new ParallelAction(
                        new SuperstructureAction(Superstructure.GoalState.STOW),
                        new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().cpFirstScoreToInsidePickup, false),
                        new SeriesAction(
                                new WaitForRegionAction(
                                        new Translation2d(3.5, -1.27),
                                        new Translation2d(6.0, 1.27)),
                                new SuperstructureAction(Superstructure.GoalState.PICKUP_LOW_CUBE, 0.0)
                        )
                )
        ));
        addAction(
                new ParallelAction(
                        new SuperstructureAction(Superstructure.GoalState.STOW),
                        new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().cpInsidePickupToSecondAlign, false),
                        new SeriesAction(
                                new WaitForRegionAction(
                                        new Translation2d(0.0, -1.27),
                                        new Translation2d(1.0, 1.27)),
                                new EnableLimelightAction(),
                                new SuperstructureAction(Superstructure.GoalState.PRE_HIGH_SCORING_CUBE, 0.0),
                                new SuperstructureAction(Superstructure.GoalState.HIGH_SCORING_CUBE),
                                new DisableLimelightAction()
                        )
                )
        );
        addAction(
                new ParallelAction(
                        new SeriesAction(
                                new SuperstructureAction(Superstructure.GoalState.SCORE),
                                new SuperstructureAction(Superstructure.GoalState.STOW, 0.0)
                        ),
                        new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().cpSecondScoreToOutsidePickup),
                        new SeriesAction(
                                new WaitForRegionAction(
                                        new Translation2d(3.5, -2.27),
                                        new Translation2d(6.5, 2.27)),
                                new SuperstructureAction(Superstructure.GoalState.PICKUP_LOW_CUBE, 0.0)
                        )
                )
        );
        if(mDockMode == AutoModeSelector.DockMode.DOCK) {
            DockActionNew dockAction = new DockActionNew(DockActionNew.StartingSide.FAR, DockActionNew.ApproachOrientation.BACK_FIRST, DockActionNew.driveDistanceStaticMetersBackFirst, 15);
            addAction(
                    new ParallelAction(
                            new SuperstructureAction(Superstructure.GoalState.STOW),
                            new InterruptableAction(new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().cpOutsidePickupToDock, false), dockAction::getInclinationAboveThresholdForDock)
                    )
            );
            addAction(dockAction);
        }
    }
}