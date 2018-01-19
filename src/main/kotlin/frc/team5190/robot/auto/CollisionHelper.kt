package frc.team5190.robot.auto

import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Waypoint
import jaci.pathfinder.modifiers.TankModifier

class CollisionHelper {
    companion object {
        fun generateNewPaths(index: Int, helper: AutoHelper?) {
//            val posX = (helper!!.trajectoryLeftDetailed[index].x + helper.trajectoryRightDetailed[index].x) / 2.0
//            val posY = (helper.trajectoryLeftDetailed[index].y + helper.trajectoryRightDetailed[index].y) / 2.0

            val posX = 10.0
            val posY = 25.0

            val waypoints = fillWaypoints(posX, posY)
            
            waypoints.add(0, Waypoint(posX, posY, 90.0))

//            waypoints.add(0, Waypoint(posX, posY, Pathfinder.d2r(NavX.angle)))

            val config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 5.0, 3.0, 60.0)
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

            newLeftList.forEach {
                println("Pos: ${it.position}, Vel: ${it.velocity}, Dur: ${it.duration}")
            }

            val combined: CombinedTrajectoryLists = mutableListOf()

            combined.add(0, newLeftList)
            combined.add(1, newRightList)

            newTrajectories = combined
        }

        private fun fillWaypoints(posX: Double, posY: Double): MutableList<Waypoint> {

            val initWaypoints = mutableListOf<Waypoint>()

            initWaypoints.add(0, Waypoint(6.0, 8.0, Pathfinder.d2r(90.0)))
            initWaypoints.add(1, Waypoint(6.0, 18.0, Pathfinder.d2r(90.0)))
            initWaypoints.add(2, Waypoint(10.0, 25.0, Pathfinder.d2r(0.0)))
            initWaypoints.add(3, Waypoint(14.0, 22.0, Pathfinder.d2r(-90.0)))

            return when {
                posY > 08.0 && posY < 18.0  -> initWaypoints.subList(1, 4)
                posY > 18.0 && posX < 10.0  -> initWaypoints.subList(2, 4)
                posY > 18.0 && posX < 14.0  -> initWaypoints.subList(3, 4)
                posY > 18.0 && posX > 14.0  -> TODO ("Don't go further")
                else                        -> initWaypoints
            }
        }

        @JvmStatic
        fun main(args: Array<String>) {
            generateNewPaths(0, null)
        }

        var newTrajectories: CombinedTrajectoryLists? = null
    }
}