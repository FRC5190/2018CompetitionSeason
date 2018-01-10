package frc.team5190.robot.navigation;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.StringTokenizer;

public class NAVHelper {

    private static final int NUM_ROWS = 54;

    public static double[][] leftPoints = new double[NUM_ROWS][3];
    public static double[][] rightPoints = new double[NUM_ROWS][3];

    public static void configPointsFromCSV(AutoMode mode) throws Exception {

        String leftPath;
        String rightPath;

        switch (mode) {
            case LEFT:
                // TODO
                leftPath = "C:\\Users\\prate\\Downloads\\testpath_left.csv";
                rightPath = "C:\\Users\\prate\\Downloads\\testpath_right.csv";
                break;

            case CENTER:
                leftPath = "C:\\Users\\prate\\Downloads\\testpath_left.csv";
                rightPath = "C:\\Users\\prate\\Downloads\\testpath_right.csv";
                break;

            case RIGHT:
                // TODO
                leftPath = "C:\\Users\\prate\\Downloads\\testpath_left.csv";
                rightPath = "C:\\Users\\prate\\Downloads\\testpath_right.csv";
                break;

            default:
                leftPath = "C:\\Users\\prate\\Downloads\\testpath_left.csv";
                rightPath = "C:\\Users\\prate\\Downloads\\testpath_right.csv";
                break;
        }
        File left = new File(leftPath);
        File right = new File(rightPath);

        int row = 0;
        int col = 0;

        BufferedReader bufferedReader = new BufferedReader(new FileReader(left));
        String line;

        while ((line = bufferedReader.readLine()) != null && row < NUM_ROWS) {
            StringTokenizer st = new StringTokenizer(line, ", ");
            while (st.hasMoreTokens()) {
                leftPoints[row][col] = Double.parseDouble(st.nextToken());
                col++;
            }
            col = 0;
            row++;
        }

        bufferedReader = new BufferedReader(new FileReader(right));
        row = 0;
        col = 0;

        while ((line = bufferedReader.readLine()) != null & row < NUM_ROWS) {
            StringTokenizer st = new StringTokenizer(line, ",");
            while (st.hasMoreTokens()) {
                rightPoints[row][col] = Double.parseDouble(st.nextToken());
                col++;
            }
            col = 0;
            row++;
        }
    }

    public enum AutoMode {
        LEFT, CENTER, RIGHT;
    }
}
