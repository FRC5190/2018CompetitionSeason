/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package frc.team5190.robot

import com.kauailabs.navx.frc.AHRS
import frc.team5190.lib.math.geometry.Pose2d
import frc.team5190.lib.math.geometry.Rotation2d
import frc.team5190.lib.math.geometry.Translation2d
import frc.team5190.lib.math.units.Distance
import frc.team5190.lib.math.units.NativeUnits
import frc.team5190.lib.utils.launchFrequency
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.sensors.Pigeon
import kotlinx.coroutines.experimental.launch
import kotlinx.coroutines.experimental.newSingleThreadContext
import kotlinx.coroutines.experimental.sync.Mutex
import kotlinx.coroutines.experimental.sync.withLock

object Localization {

    private val localizationContext = newSingleThreadContext("Localization")
    private val localizationMutex = Mutex()

    var robotPosition = Pose2d()
        private set

    private var prevL: Distance = NativeUnits(0)
    private var prevR: Distance = NativeUnits(0)
    private var prevA = Rotation2d()

    init {
        launch(localizationContext) { reset() }
        launchFrequency(100, localizationContext) { run() }
    }

    suspend fun reset(pose: Pose2d = Pose2d(Translation2d(), Rotation2d())) = localizationMutex.withLock {
        robotPosition = pose
        prevL = NativeUnits(DriveSubsystem.falconDrive.leftEncoderPosition)
        prevR = NativeUnits(DriveSubsystem.falconDrive.rightEncoderPosition)
        prevA = Rotation2d.fromDegrees(Pigeon.correctedAngle)
    }

    private suspend fun run() = localizationMutex.withLock {
        val posL = NativeUnits(DriveSubsystem.falconDrive.leftEncoderPosition)
        val posR = NativeUnits(DriveSubsystem.falconDrive.rightEncoderPosition)

        val angA = Rotation2d.fromDegrees(Pigeon.correctedAngle)

        val deltaL = posL - prevL
        val deltaR = posR - prevR
        val deltaA = angA - prevA

        val kinematics = Kinematics.forwardKinematics(deltaL.FT, deltaR.FT, deltaA.radians)
        robotPosition = robotPosition.transformBy(Pose2d.fromTwist(kinematics))

        prevL = posL
        prevR = posR
        prevA = angA
    }
}
