package frc.team5190.robot.auto

import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Waypoint
import jaci.pathfinder.modifiers.TankModifier

class CollisionHelper {
    companion object {
        fun generateNewPaths(index: Int, helper: AutoHelper) {
            val posX = (helper.trajectoryLeftDetailed[index].x + helper.trajectoryRightDetailed[index].x) / 2.0
            val posY = (helper.trajectoryLeftDetailed[index].y + helper.trajectoryRightDetailed[index].y) / 2.0

            val waypoints = arrayOfNulls<Waypoint>(2)

            waypoints[0] = Waypoint(posX, posY, Pathfinder.d2r(90.0))
            waypoints[1] = Waypoint(posX, posY + 5, Pathfinder.d2r(90.0))

            val config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 20.0, 5.0, 3.0, 60.0)
            val trajectory = Pathfinder.generate(waypoints, config)

            val tankModifier = TankModifier(trajectory).modify(25.0 / 12.0)

            val left = tankModifier.leftTrajectory
            val right = tankModifier.rightTrajectory

            val newLeftList: TrajectoryList = mutableListOf()
            val newRightList: TrajectoryList = mutableListOf()

            for (i in 0 until left.length()) {
                val seg = left.get(i)
                newLeftList[i] = TrajectoryData(seg.position, seg.velocity, (seg.dt * 1000).toInt())
            }

            for (i in 0 until right.length()) {
                val seg = right.get(i)
                newRightList[i] = TrajectoryData(seg.position, seg.velocity, (seg.dt * 1000).toInt())
            }

            val combined: CombinedTrajectoryLists = mutableListOf()

            combined[0] = newLeftList
            combined[1] = newRightList

            newTrajectories = combined
        }

        var newTrajectories: CombinedTrajectoryLists? = null

    }
}