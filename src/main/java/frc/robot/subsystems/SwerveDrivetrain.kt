package frc.robot.subsystems

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.MathSharedStore
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Vector
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import frc.robot.OI
import frc.robot.generated.TunerConstants
import frc.robot.lib.component1
import frc.robot.lib.component2
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.lib.meters
import frc.robot.lib.milliseconds
import frc.robot.lib.radians
import frc.robot.subsystems.SwerveDrivetrain.SwerveState.*
import java.util.Optional
import kotlin.math.abs
import kotlin.math.sign
import org.littletonrobotics.junction.Logger

object SwerveDrivetrain :
    TunerConstants.TunerSwerveDrivetrain(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight,
    ) {

    enum class SwerveState {
        FieldCentric,
        XBrake,
    }

    var currentState = FieldCentric

    private val fieldCentricRequest =
        SwerveRequest.FieldCentric()
            .withDriveRequestType(Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position)
            .withRotationalDeadband(.05)

    fun stateActions() {
        if (currentState != FieldCentric) {
            headingTimer.reset()
        }
        when (currentState) {
            FieldCentric -> {
                var rot = OI.rotInput() * MAX_ROT_SPEED
                if (rot == 0.0 && headingTimer.hasElapsed(350.milliseconds)) {
                    rot = headingPIDController.calculate(state.Pose.rotation.radians, oldHeading)
                } else if (rot != 0.0) {
                    headingTimer.reset()
                } else {
                    oldHeading = state.Pose.rotation.radians
                }
                val (x, y) = OI.driveInput().times(MAX_SPEED)
                val limited = translationLimiter.calculate(MAX_SPEED * x, MAX_SPEED * y)
                setControl(
                    fieldCentricRequest
                        .withVelocityX(limited.x)
                        .withVelocityY(limited.y)
                        .withRotationalRate(rot)
                )
            }

            XBrake -> {
                brake()
            }
        }
    }

    val headingTimer = Timer().also { it.start() }
    var oldHeading: Double = 0.0

    fun stateTransitions() {
        if (OI.resetRobotYaw()) {
            headingTimer.reset()
        }
        when (currentState) {
            FieldCentric -> {
                if (OI.xBrake()) {
                    currentState = XBrake
                }
            }

            XBrake -> {
                if (!OI.xBrake()) {
                    currentState = FieldCentric
                }
            }
        }
    }

    private val posePIDController = PIDController(5.0, 0.0, 0.5)
    private val headingPIDController =
        PIDController(5.0, 0.0, 0.15).apply { enableContinuousInput(-Math.PI, Math.PI) }

    private val translationLimiter = TranslationSlewRateLimiter(15.0)

    private val FieldCentricFacingAngleAlignments: SwerveRequest.FieldCentricFacingAngle =
        SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(5.0, 0.0, 0.15)
            .withRotationalDeadband(0.05)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position)
            .apply { HeadingController.setTolerance(5.degrees.radians) }

    private val rotateAroundPointRequest: SwerveRequest.FieldCentric =
        SwerveRequest.FieldCentric()
            .withRotationalDeadband(0.05)
            .withDriveRequestType(Velocity)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)

    /**
     * Drives to a pose using a PID controller
     *
     * @param target the desired Pose2d to drive to
     * @param threshold distance from target to consider "at target", in meters
     * @param alwaysMaxSpeed whether the robot should go at max speed for the entire motion
     * @param minSpeed the slowest the robot will move in m/s if fullSpeed is false
     * @return true if distance to target is < threshold
     */
    fun driveToPose(
        target: Pose2d,
        threshold: Distance = 0.inches,
        maxSpeed: Double = MAX_SPEED,
        alwaysMaxSpeed: Boolean = false,
        minSpeed: Double = -MAX_SPEED,
    ): Boolean {
        Logger.recordOutput("DriveToPose target", target)
        val diff = target.translation - state.Pose.translation
        FieldCentricFacingAngleAlignments.withTargetDirection(target.rotation)

        // Because the error is always positive, our controller output is always negative.
        // Inverting here allows us to work with a positive speed, and the direction is controlled
        // by diff.angle
        val output = -posePIDController.calculate(diff.norm)
        val clampedOutput =
            if (alwaysMaxSpeed) maxSpeed else MathUtil.clamp(output, minSpeed, maxSpeed)

        // Use unit vector of diff to create chassis speeds
        val unit = diff / diff.norm
        val speeds = unit * clampedOutput

        val limited = translationLimiter.calculate(speeds)

        setControl(
            FieldCentricFacingAngleAlignments.withVelocityX(limited.x).withVelocityY(limited.y)
        )

        return diff.norm < threshold.meters
    }

    fun rotateAroundPoint(
        targetHeading: Rotation2d,
        centerOfRotation: Translation2d,
        threshold: Angle = 0.radians,
        maxSpeed: Double = MAX_SPEED,
        alwaysMaxSpeed: Boolean = false,
    ): Boolean {
        val rawError = (targetHeading - state.Pose.rotation).radians
        // Center of rotation on the right (negative Y) means clockwise to go forward, and vice
        // versa
        val direction = sign(centerOfRotation.y)
        val radius = centerOfRotation.norm
        val maxRotation = maxSpeed / radius
        val headingOutput = headingPIDController.calculate(abs(rawError))
        val clampedRotation =
            if (alwaysMaxSpeed) direction * maxRotation
            else direction * MathUtil.clamp(abs(headingOutput), 0.0, maxRotation)

        setControl(
            rotateAroundPointRequest
                .withRotationalRate(clampedRotation)
                .withCenterOfRotation(centerOfRotation)
        )

        return abs(rawError).radians < threshold
    }

    private lateinit var simNotifier: Notifier
    private var lastSimTime = 0.0

    /* Keep track if we've ever applied the operator perspective before or not */
    private var hasAppliedOperatorPerspective = false

    init {
        if (Utils.isSimulation()) {
            startSimThread()
        }
    }

    fun periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent {
                setOperatorPerspectiveForward(
                    if (it == Red) kRedAlliancePerspectiveRotation
                    else kBlueAlliancePerspectiveRotation
                )
                hasAppliedOperatorPerspective = true
            }
        }

        Logger.recordOutput("swerve/state", currentState)
        Logger.recordOutput("swerve/lockedHeader", oldHeading)
    }

    private fun startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds()

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = Notifier {
            val currentTime = Utils.getCurrentTimeSeconds()
            val deltaTime = currentTime - lastSimTime
            lastSimTime = currentTime

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage())
        }
        simNotifier.startPeriodic(kSimLoopPeriod)
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    override fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds))
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * Note that the vision measurement standard deviations passed into this method will continue to
     * apply to future measurements until a subsequent call to [.setVisionMeasurementStdDevs] or
     * this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the
     *   form [x, y, theta]ᵀ, with units in meters and radians.
     */
    override fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3, N1>,
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(timestampSeconds),
            visionMeasurementStdDevs,
        )
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The timestamp of the pose in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    override fun samplePoseAt(timestampSeconds: Double): Optional<Pose2d> {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds))
    }

    val idleRequest = SwerveRequest.Idle()

    fun idle() {
        setControl(idleRequest)
    }

    val xBrakeRequest = SwerveRequest.SwerveDriveBrake()

    fun brake() {
        setControl(xBrakeRequest)
    }

    private const val kSimLoopPeriod = 0.004 // 4 ms

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private val kBlueAlliancePerspectiveRotation: Rotation2d = Rotation2d.kZero

    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private val kRedAlliancePerspectiveRotation: Rotation2d? = Rotation2d.k180deg

    // Speeds are in SI units
    const val MAX_SPEED: Double = 3.79
    const val MAX_ROT_SPEED = 2 * Math.PI

    /** Slew rate limiter on a 2D velocity vector. */
    private class TranslationSlewRateLimiter(private val rateLimit: Double) {
        private var prev = Translation2d()
        private var prevTime = MathSharedStore.getTimestamp()

        fun calculate(next: Translation2d): Translation2d {
            val currentTime = MathSharedStore.getTimestamp()
            val dt = currentTime - prevTime
            prevTime = currentTime
            prev = MathUtil.slewRateLimit(prev, next, dt, rateLimit)
            return prev
        }

        fun calculate(vector: Vector<N2>) = calculate(Translation2d(vector))

        fun calculate(x: Double, y: Double) = calculate(Translation2d(x, y))
    }
}
