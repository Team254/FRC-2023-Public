package com.team254.lib.vision;

import java.util.ArrayList;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

public class VisionVarianceChecker {
    ArrayList<Double> xMeasurements = new ArrayList<Double>();
    ArrayList<Double> yMeasurements = new ArrayList<Double>();
    ArrayList<Double> thetaMeasurements = new ArrayList<Double>();
    private double sumX;
    private double sumY;
    private double sumTheta;
    private int maxSize;


    public VisionVarianceChecker(int maxSize) {
        this.maxSize = maxSize;
        sumX = 0;
        sumY = 0;
        sumTheta = 0;
    }

    public boolean isValid(Translation2d offset, double rotationOffset, double stdX, double stdY, double stdTheta) {
        if (xMeasurements.size() < maxSize) {
            return true;
        }
        double meanX = sumX / xMeasurements.size();
        double meanY = sumY / yMeasurements.size();
        double meanTheta = sumTheta / thetaMeasurements.size();



        double diffMeanX = Math.abs(meanX - offset.x());
        double diffMeanY = Math.abs(meanY - offset.y());
        double diffMeanTheta = Math.abs(meanTheta - rotationOffset);

        if (diffMeanX < stdX && diffMeanY < stdY && diffMeanTheta < stdTheta) {
            return true;
        }

        return false;
 
    }

    public void add(Translation2d offset, double rotationOffset) {
        sumX += offset.x();
        sumY += offset.y();
        sumTheta += rotationOffset;

        xMeasurements.add(offset.x());
        yMeasurements.add(offset.y());
        thetaMeasurements.add(rotationOffset);
        if (xMeasurements.size() > maxSize) {
            sumX -= xMeasurements.remove(0);
        }
        if (yMeasurements.size() > maxSize) {
            sumY -= yMeasurements.remove(0);
        }
        if (thetaMeasurements.size() > maxSize) {
            sumTheta -= thetaMeasurements.remove(0);
        }
    }


    public int getSize() {
        return xMeasurements.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    public void clear() {
        xMeasurements.clear();
        yMeasurements.clear();
        thetaMeasurements.clear();
    }
    
}
