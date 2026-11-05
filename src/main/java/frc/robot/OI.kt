package frc.robot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.wpilibj.XboxController
import frc.robot.lib.signSquare

object OI {

    val driveController = XboxController(0)

    val sysIdController = XboxController(2)

    fun resetRobotYaw() = driveController.startButton

    fun xBrake() = driveController.xButton

    // Swerve drive

    fun driveInput() =
        MathUtil.copyDirectionPow(
            MathUtil.applyDeadband(
                VecBuilder.fill(-driveController.leftY, driveController.leftX),
                0.1,
            ),
            2.0,
        )

    fun rotInput() = -driveController.rightX.signSquare

    fun sysIdFQ() = sysIdController.aButton

    fun sysIdRQ() = sysIdController.bButton

    fun sysIdFD() = sysIdController.xButton

    fun sysIdRD() = sysIdController.yButton
}
