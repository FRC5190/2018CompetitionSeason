package frc.team5190.robot.auto

import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Waypoint
import jaci.pathfinder.modifiers.TankModifier

class CollisionHelper {
    companion object {
        fun generateNewPaths(index: Int, helper: AutoHelper) {

            val waypoints = fillWaypoints(index, helper)

            val config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 5.0, 3.0, 60.0)
            val trajectory = Pathfinder.generate(waypoints.toTypedArray(), config)

            val tankModifier = TankModifier(trajectory).modify(25.0 / 12.0)

            val left = tankModifier.leftTrajectory
            val right = tankModifier.rightTrajectory

            val newLeftList: TrajectoryList = mutableListOf()
            val newRightList: TrajectoryList = mutableListOf()

            for (i in 0 until left.length()) {
                val segLeft = left.get(i)
                newLeftList.add(i, TrajectoryData(segLeft.position, segLeft.velocity, (segLeft.dt * 1000).toInt()))

                val segRight = right.get(i)
                newRightList.add(i, TrajectoryData(segRight.position, segRight.velocity, (segRight.dt * 1000).toInt()))
            }

            println("New trajectory computation completed.")

            newLeftList.forEach {
                println("Pos: ${it.position}, Vel: ${it.velocity}, Dur: ${it.duration}")
            }

            val combined: CombinedTrajectoryLists = mutableListOf()

            combined.add(0, newLeftList)
            combined.add(1, newRightList)

            newTrajectories = combined
        }

        private fun fillWaypoints(index: Int, helper: AutoHelper): MutableList<Waypoint> {

            val waypoints = mutableListOf<Waypoint>()

            for (i in index until helper.trajectoryLeftDetailed.size step 10) {


                val posX  = (helper.trajectoryLeftDetailed[i].x + helper.trajectoryRightDetailed[i].x) / 2.0
                val posY  = (helper.trajectoryLeftDetailed[i].y + helper.trajectoryRightDetailed[i].y) / 2.0
                val angle = (helper.trajectoryLeftDetailed[i].h + helper.trajectoryRightDetailed[i].h) / 2.0

                if (i == 0) {
                    println("X: $posX, Y: $posY, H: $angle")
                }

                waypoints.add(Waypoint(posX, posY, Pathfinder.d2r(Pathfinder.r2d(angle))))
            }

            return waypoints
        }

        var newTrajectories: CombinedTrajectoryLists? = null
    }
}