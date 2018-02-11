package frc.team5190.robot.listener

import edu.wpi.first.wpilibj.networktables.NetworkTable
import edu.wpi.first.wpilibj.command.Subsystem
import jaci.pathfinder.Trajectory
import java.io.ObjectInputStream
import java.io.ByteArrayInputStream
import java.util.*

object ListenerSubsystem : Subsystem() {

    private val pathfinderInputTable: NetworkTable = NetworkTable.getTable("pathfinderInput")
    private var genId = 0

    override fun initDefaultCommand() {}

    fun getPath(path:String, obstructed: Boolean, index: Double) {
        try {
            pathfinderInputTable.putNumber("getTrajectory", genId.toDouble())
            pathfinderInputTable.putString("path", "LLX")
            pathfinderInputTable.putBoolean("obstructed", false)
            pathfinderInputTable.putNumber("index", 0.0)
            NetworkTable.flush()
            genId++
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    fun deserializeTrajectoryArray(serializedTrajectoryArray: String): Array<Trajectory>? {
        var trajectories: Array<Trajectory>? = null
        try {
            val b = Base64.getDecoder().decode(serializedTrajectoryArray.toByteArray())
            val bi = ByteArrayInputStream(b)
            val si = ObjectInputStream(bi)
            trajectories = si.readObject() as Array<Trajectory>
        } catch (e: Exception) {
            e.printStackTrace()
        }

        return trajectories
    }
}
