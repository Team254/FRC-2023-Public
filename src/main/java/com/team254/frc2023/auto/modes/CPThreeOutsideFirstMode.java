package com.team254.frc2023.auto.modes;

import com.team254.frc2023.AutoModeSelector;
import com.team254.frc2023.auto.AutoModeEndedException;
import com.team254.frc2023.auto.actions.*;
import com.team254.frc2023.subsystems.Drive;
import com.team254.frc2023.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

/*
* Cable Protector/Bump Side - Scores High Cone, picks up outside cube, scores it high, picks up inside cube scores mid.
* */
public class CPThreeOutsideFirstMode extends AutoModeBase {

    public CPThreeOutsideFirstMode(AutoModeSelector.DockMode dockMode) {
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
                        new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().cpFirstScoreToOutsidePickup, false),
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
                        new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().cpOutsidePickupToSecondAlign, false),
                        new SeriesAction(
                                new WaitForRegionAction(
                                        new Translation2d(0.0, -1.27),
                                        new Translation2d(4.0, 1.27)),
                                new EnableLimelightAction(),
                                new WaitForRegionAction(
                                        new Translation2d(0.0, -1.27),
                                        new Translation2d(2.5, 1.27)),
                                new DisableLimelightAction(),
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
                            new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().cpSecondScoreToInsidePickup),
                            new SeriesAction(
                                    new WaitForRegionAction(
                                            new Translation2d(3.5, -2.27),
                                            new Translation2d(6.5, 2.27)),
                                    new SuperstructureAction(Superstructure.GoalState.PICKUP_LOW_CUBE, 0.0)
                            )
                    )
        );
        addAction(
            new ParallelAction(
                    new SuperstructureAction(Superstructure.GoalState.STOW),
                    new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().cpInsidePickupToThirdScore, false),
                    new SeriesAction(
                            new WaitForRegionAction(
                                    new Translation2d(0.0, -1.27),
                                    new Translation2d(4.0, 1.27)),
                            new EnableLimelightAction(),
                            new WaitForRegionAction(
                                    new Translation2d(0.0, -1.27),
                                    new Translation2d(2.5, 1.27)),
                            new DisableLimelightAction(),
                            new WaitForRegionAction(new Translation2d(0.0, -1.27), new Translation2d(1.0, 1.27)),
                            new EnableLimelightAction(),
                            new WaitForRegionAction(new Translation2d(0.0, -1.27), new Translation2d(0.6, 1.27)),
                            new DropAction(),
                            new DisableLimelightAction()
                    )
            )
        );

        addAction(new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().cpThirdScoreToBackoff, false));
    }
}