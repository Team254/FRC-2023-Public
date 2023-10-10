package com.team254.lib.geometry;

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    Pose2d getPose();

    S transformBy(Pose2d transform);

    S mirror();
}
