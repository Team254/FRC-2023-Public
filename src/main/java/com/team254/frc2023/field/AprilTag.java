package com.team254.frc2023.field;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

public class AprilTag {

    private int id;
    private double height;
    private Pose2d fieldToTag;
    private boolean isScoring;

    private Translation2d tagToLeftAlign;

    private Translation2d tagToRightAlign;

    private Translation2d tagToCenterAlign;

    public AprilTag(int id, double height, Pose2d fieldToTag, boolean isScoring, Translation2d tagToCenterAlign, Translation2d tagToLeftAlign, Translation2d tagToRightAlign) {
        this.id = id;
        this.height = height;
        this.fieldToTag = fieldToTag;
        this.isScoring = isScoring;
        this.tagToCenterAlign = tagToCenterAlign;
        this.tagToLeftAlign = tagToLeftAlign;
        this.tagToRightAlign = tagToRightAlign;
    }

    public int getId() {
        return id;
    }

    public double getHeight() {
        return height;
    }

    public Pose2d getFieldToTag() {
        return fieldToTag;
    }

    public boolean isScoring() {
        return isScoring;
    }

    public Translation2d getTagToCenterAlign() {
        return tagToCenterAlign;
    }

    public Translation2d getTagToLeftAlign() {
        return tagToLeftAlign;
    }

    public Translation2d getTagToRightAlign() {
        return tagToRightAlign;
    }
}
