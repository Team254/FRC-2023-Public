package com.team254.frc2023.limelight;


import com.team254.frc2023.subsystems.Limelight;
import org.junit.jupiter.api.Test;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.ArrayList;

public class UndistortMapFileWriterTest {

    // START : Params for generation
    private static final boolean DO_WRITE_UNDISTORT_MAP = false;
    // SET THESE 2 things
    final String limelightId = "B";
    private static final int kWidth = 640;
    private static final int kHeight = 480;

    // END : Params for generation


    @Test
    public void testMakeUndistortMap() throws Exception {
        // Don't edit below this
        if (!DO_WRITE_UNDISTORT_MAP) {
            return;
        }

        final int width = kWidth;
        final int height = kHeight;

        String varName = "map";
        StringBuilder sb = new StringBuilder();

        String className = String.format("UndistortMap_Limelight_%s_%dx%d", limelightId, width, height);
        // Header
        sb.append("package com.team254.frc2023.limelight.undistort.precomputedmaps;\n" +
                "\n" +
                "import com.team254.frc2023.limelight.undistort.UndistortMap;\n" +
                "import com.team254.lib.util.Util;\n" +
                "\n" +
                String.format("public class %s implements UndistortMap {\n", className));

        // Write overall variable
        String variableLine = String.format("  double[][][] %s = new double[%d][%d][2];\n", varName, width, height);
        sb.append(variableLine);

        // Write Y column loader methods
        for (int x = 0; x < width; x++) {
            sb.append(String.format("  public double[][] loadCol%03d() { ", x));
            ArrayList<String> lineParts = new ArrayList<>();
            for (int y = 0; y < height; y++) {
                double[] normalizedPt =  Limelight.getInstance().undistortFromOpenCV(new double[] { x * 1.0 / width ,  y * 1.0 / height });
                String strPt = String.format("{%.3f,%.3f}", normalizedPt[0], normalizedPt[1]);
                lineParts.add(strPt);
            }
            String numbers = String.join(",", lineParts);
            sb.append(String.format("return new double[][]{%s};", numbers));
            sb.append(" }\n");
        }

        // Write static loader
        sb.append("\n");
        sb.append(String.format("  public %s() {\n", className));
        for (int x = 0; x < width; x++) {
            String loadLine = String.format("    map[%d] = loadCol%03d();\n", x, x);
            sb.append(loadLine);
        }
        // End static loader
        sb.append("  }\n");

        // Write getters
        // normalizedToUndistortedNormalized
        sb.append("\n");
        sb.append("  @Override\n");
        sb.append("  public double[] normalizedToUndistortedNormalized(double x, double y) {\n");
        sb.append("    int denormalizedX = (int) (x * pixelWidth());\n");
        sb.append("    int denormalizedY = (int) (y * pixelHeight());\n");
        sb.append("    return pixelToUndistortedNormalized(denormalizedX, denormalizedY);\n");
        sb.append("  }\n\n");

        // pixelToUndistortedNormalized
        sb.append("  @Override\n");
        sb.append("  public double[] pixelToUndistortedNormalized(int x, int y) {\n");
        sb.append("    return map[Util.limit(x, 0, pixelWidth() - 1)][Util.limit(y, 0, pixelHeight() - 1)];\n");
        sb.append("  }\n\n");

        // pixelWidth
        sb.append("  @Override\n");
        sb.append("  public int pixelWidth() {\n");
        sb.append("    return " + width + ";\n");
        sb.append("  }\n\n");

        // pixelHeight
        sb.append("  @Override\n");
        sb.append("  public int pixelHeight() {\n");
        sb.append("    return " + height + ";\n");
        sb.append("  }\n\n");

        sb.append("}\n");

        String filePath = String.format("/tmp/%s.java", className);
        BufferedWriter writer = new BufferedWriter(new FileWriter(filePath));
        writer.write(sb.toString());
        writer.close();
        System.out.println("Wrote " + filePath);
    }
}
