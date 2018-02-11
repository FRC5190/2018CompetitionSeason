package frc.team5190.robot.listener;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

import java.io.ByteArrayInputStream;
import java.io.ObjectInputStream;
import java.util.Base64;

public class Listener implements ITableListener {

    private NetworkTable pathfinderOutputTable;

    private Listener() {
        NetworkTable.setClientMode();
        NetworkTable.setTeam(5190);
        NetworkTable.setIPAddress("10.51.90.2");
        NetworkTable.initialize();
        pathfinderOutputTable = NetworkTable.getTable("pathfinderOutput");
        pathfinderOutputTable.addTableListener(this, true);
    }

    @Override
    public void valueChanged(ITable iTable, String string, Object receivedObject, boolean newValue) {
        if (string.equals("result")) {
            Trajectory[] trajectories = deserializeTrajectoryArray((String) pathfinderOutputTable.getString("trajectories", ""));
            int result = (int) pathfinderOutputTable.getNumber("result", -1);
            String path = pathfinderOutputTable.getString("path", "LLX");
            System.out.println("Got path: " + result + ", " + path);
            for (int i = 0; i < trajectories[0].segments.length; i++) {
                System.out.printf("%d) x: %.2f y: %.2f heading: %.2f velocity: %.2f acceleration: %.2f jerk: %.2f \n" + "", i, trajectories[0].segments[i].x, trajectories[0].segments[i].y, Pathfinder.r2d(trajectories[0].segments[i].heading), trajectories[0].segments[i].velocity, trajectories[0].segments[i].acceleration, trajectories[0].segments[i].jerk);
            }
        }
    }

    private static Trajectory[] deserializeTrajectoryArray(String serializedTrajectoryArray) {
        Trajectory[] trajectories = null;
        try {
            System.out.println(serializedTrajectoryArray);
            byte[] b = Base64.getDecoder().decode(serializedTrajectoryArray.getBytes());
            ByteArrayInputStream bi = new ByteArrayInputStream(b);
            ObjectInputStream si = new ObjectInputStream(bi);
            trajectories = (Trajectory[]) si.readObject();
        } catch (Exception e) {
            e.printStackTrace();
        }

        return trajectories;
    }

    public static Listener INSTANCE = new Listener();
}