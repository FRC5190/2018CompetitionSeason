package frc.team5190.robot.auto

import org.junit.Test

class AutonomousTimingTest {
    @Test
    fun testTiming() {
        println("Left Start to Near Scale: ${FastTrajectories.leftStartToNearScale.lastState.t} seconds.")
        println("Left Start to Far Scale:  ${FastTrajectories.leftStartToFarScale.lastState.t} seconds.")
    }
}