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

            val waypoints = fillWaypoints(posX, posY).toTypedArray()

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

        private fun fillWaypoints(posX: Double, posY: Double): MutableList<Waypoint> {

            val initWaypoints = mutableListOf<Waypoint>()

            initWaypoints[0] = Waypoint(6.0, 8.0, Pathfinder.d2r(90.0))
            initWaypoints[1] = Waypoint(6.0, 18.0, Pathfinder.d2r(90.0))
            initWaypoints[2] = Waypoint(10.0, 25.0, Pathfinder.d2r(0.0))
            initWaypoints[3] = Waypoint(14.0, 22.0, Pathfinder.d2r(-90.0))

            return when {
                posY > 08.0 && posY < 18.0  -> initWaypoints.subList(1, 4)
                posY > 18.0 && posX < 10.0  -> initWaypoints.subList(2, 4)
                posY > 18.0 && posX < 14.0  -> initWaypoints.subList(3, 4)
                posY > 18.0 && posX > 14.0  -> TODO ("Don't go further")
                else                        -> initWaypoints
            }
        }

        var newTrajectories: CombinedTrajectoryLists? = null
    }
}