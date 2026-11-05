package frc.robot

// import frc.robot.auto.RBRBOC

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.net.WebServer
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import frc.robot.auto.*
import frc.robot.generated.Telemetry
import frc.robot.lib.hz
import frc.robot.subsystems.SwerveDrivetrain
import frc.robot.subsystems.SwerveDrivetrain.headingTimer
import frc.robot.subsystems.elevator
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher

/**
 * The functions in this object (which basically functions as a singleton class) are called
 * automatically corresponding to each mode, as described in the TimedRobot documentation. This is
 * written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also
 * update the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when
 * renaming the object or package, it will get changed everywhere.)
 */
object Robot : LoggedRobot() {
    init {
        // Kotlin initializer block, which effectually serves as the constructor code.
        // https://kotlinlang.org/docs/classes.html#constructors
        // This work can also be done in the inherited `robotInit()` method. But as of the 2025
        // season the
        // `robotInit` method's Javadoc encourages using the constructor and the official templates
        // moved initialization code out `robotInit` and into the constructor. We follow suit in
        // Kotlin.

        // Report the use of the Kotlin Language for "FRC Usage Report" statistics.
        // Please retain this line so that Kotlin's growing use by teams is seen by FRC/WPI.
        HAL.report(
            tResourceType.kResourceType_Language,
            tInstances.kLanguage_Kotlin,
            0,
            WPILibVersion.Version,
        )
        DriverStation.silenceJoystickConnectionWarning(true)
        Logger.recordMetadata("GitBranch", GIT_BRANCH)
        Logger.recordMetadata("GitSHA", GIT_SHA)
        Logger.addDataReceiver(NT4Publisher())
        Logger.start()
        if (isReal()) {
            // DataLog will automatically log all NT changes. AKit logs to NT, DataLog logs NT to
            // file
            DataLogManager.start()
            SignalLogger.setPath("/U/ctre-logs")
            SignalLogger.start()
        }
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog())
        SwerveDrivetrain.registerTelemetry(Telemetry::telemeterize)
        Notifier {
                if (DriverStation.isEnabled()) {
                    DataLogManager.getLog().resume()
                }
            }
            .startPeriodic(1.hz)
        WebServer.start(5800, Filesystem.getDeployDirectory().path)
    }

    override fun robotPeriodic() {

        SwerveDrivetrain.periodic()

        // Execute actions for active states in all subsystems except drivetrain
        elevator.state_transitions()
        elevator.state_actions()

        // Update pose on Elastic
        field2d.robotPose = SwerveDrivetrain.state.Pose
    }

    override fun autonomousInit() {
        autoStartTimer.restart()
        autoSelector.selected?.init()
    }

    private val autoSelector =
        SendableChooser<Auto>().apply { SmartDashboard.putData("Auto Mode", this) }

    val autoStartTimer = Timer()

    override fun autonomousPeriodic() {
        autoSelector.selected?.run()
    }

    override fun teleopInit() {
        headingTimer.reset()
    }

    override fun teleopPeriodic() {

        // Update state machine transitions
        SwerveDrivetrain.stateTransitions()

        // Auto uses driveToPose() separately of state-based actions,
        //  so we only need to do these for the drivetrain in teleop
        SwerveDrivetrain.stateActions()
    }

    private val field2d = Field2d()

    override fun disabledInit() {
        SmartDashboard.putData(field2d)
        // Re-enabled brake mode on motors by sending an Idle request
        SwerveDrivetrain.idle()
    }

    override fun simulationInit() {
        Logger.recordOutput("ZeroedPoses", Pose3d(), Pose3d())
    }
}
