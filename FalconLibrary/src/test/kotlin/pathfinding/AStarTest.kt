import frc.team5190.lib.math.geometry.Pose2d
import frc.team5190.lib.math.geometry.Pose2dWithCurvature
import frc.team5190.lib.math.geometry.Rotation2d
import frc.team5190.lib.math.geometry.Translation2d
import frc.team5190.lib.math.trajectory.AStarOptimizer
import frc.team5190.lib.math.trajectory.timing.CentripetalAccelerationConstraint
import frc.team5190.lib.math.trajectory.timing.TimingConstraint

fun main(args: Array<String>) {
    PathWindow

    // ROBOT DIMENSIONS
    val kRobotWidth = 27.0 / 12.0
    val kRobotLength = 33.0 / 12.0
    val kIntakeLength = 14.0 / 12.0
    val kBumperLength = 04.0 / 12.0

    // ROBOT POSES
    val kRobotStartX = (kRobotLength / 2.0) + kBumperLength

    val kExchangeZoneBottomY = 14.5
    val kPortalZoneBottomY = 27 - (29.69 / 12.0)

    val kRobotSideStartY = kPortalZoneBottomY - (kRobotWidth / 2.0) - kBumperLength - 4.0 / 12.0
    val kRobotCenterStartY = kExchangeZoneBottomY - (kRobotWidth / 2.0) - kBumperLength

    // MECHANISM TRANSFORMATIONS
    val kCenterToIntake = Pose2d(Translation2d(-(kRobotLength / 2.0) - kIntakeLength, 0.0), Rotation2d())
    val kCenterToFrontBumper = Pose2d(Translation2d(-(kRobotLength / 2.0) - kBumperLength, 0.0), Rotation2d())

    // Constants in Feet Per Second
    val kMaxVelocity = 8.0
    val kMaxAcceleration = 4.0
    val kMaxCentripetalAcceleration = 4.5

    // Constraints
    val kConstraints = arrayListOf<TimingConstraint<Pose2dWithCurvature>>(
            CentripetalAccelerationConstraint(kMaxCentripetalAcceleration))

    // Field Relative Constants
    val kSideStart = Pose2d(Translation2d(kRobotStartX, kRobotSideStartY), Rotation2d(-1.0, 0.0))
    val kCenterStart = Pose2d(Translation2d(kRobotStartX, kRobotCenterStartY), Rotation2d())

    val kNearScaleEmpty = Pose2d(Translation2d(23.95, 20.2), Rotation2d.fromDegrees(160.0))
    val kNearScaleFull = Pose2d(Translation2d(23.95, 20.0), Rotation2d.fromDegrees(170.0))
    val kNearScaleFullInner = Pose2d(Translation2d(24.3, 20.0), Rotation2d.fromDegrees(170.0))

    val kNearCube1 = Pose2d(Translation2d(16.5, 19.2), Rotation2d.fromDegrees(190.0))
    val kNearCube2 = Pose2d(Translation2d(17.4, 15.0), Rotation2d.fromDegrees(245.0))
    val kNearCube3 = Pose2d(Translation2d(17.6, 14.5), Rotation2d.fromDegrees(245.0))

    val kNearCube1Adjusted = kNearCube1.transformBy(kCenterToIntake)
    val kNearCube2Adjusted = kNearCube2.transformBy(kCenterToIntake)
    val kNearCube3Adjusted = kNearCube3.transformBy(kCenterToIntake)

    val kFarCube1 = Pose2d(Translation2d(16.5, 20.2), Rotation2d.fromDegrees(190.0))
    val kFarCube1Adjusted = kFarCube1.transformBy(kCenterToIntake)

    val kSwitchLeft = Pose2d(Translation2d(11.9, 18.5), Rotation2d())
    val kSwitchRight = Pose2d(Translation2d(11.9, 08.5), Rotation2d())

    val kSwitchLeftAdjusted = kSwitchLeft.transformBy(kCenterToFrontBumper)
    val kSwitchRightAdjusted = kSwitchRight.transformBy(kCenterToFrontBumper)

    val kFrontPyramidCube = Pose2d(Translation2d(10.25, 13.5), Rotation2d())
    val kFrontPyramidCubeAdjusted = kFrontPyramidCube.transformBy(kCenterToIntake)

    val aStarTest = AStarOptimizer(
            PathWindow.ROBOT_SIZE,
            PathWindow.LEFT_SWITCH, PathWindow.PLATFORM, PathWindow.RIGHT_SWITCH
    )


    PathWindow.path = arrayListOf(kFrontPyramidCubeAdjusted,
            kFrontPyramidCubeAdjusted.transformBy(Pose2d(Translation2d(2.0, 8.0), Rotation2d.fromDegrees(180.0))),
            kFrontPyramidCubeAdjusted.transformBy(Pose2d(Translation2d(7.0, 8.0), Rotation2d.fromDegrees(180.0))),
            kNearScaleEmpty)

    //AStarTest.start = Point(0.0, 2.5)
    //AStarTest.goal = Point( PathWindow.FIELD_LENGTH - 23.7, 20.2)

    //val startAngle = 0.0
    //val endAngle = -165.0

    //PathWindow.bannedAreas += Rectangle(0.0, PathWindow.LEFT_SWITCH.y, PathWindow.LEFT_SWITCH.x, PathWindow.LEFT_SWITCH.h)

    /*
    PathWindow.bannedAreas += Rectangle(PathWindow.LEFT_SWITCH.x + PathWindow.LEFT_SWITCH.w,
            PathWindow.PLATFORM.y + PathWindow.PLATFORM.h,
            PathWindow.FIELD_LENGTH / 2.0 - PathWindow.LEFT_SWITCH.w - PathWindow.LEFT_SWITCH.x,
            PathWindow.FIELD_WIDTH - PathWindow.PLATFORM.y - PathWindow.PLATFORM.h)
*/

    //println(path)

}