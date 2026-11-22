package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.measure.Distance
import frc.robot.OI
import frc.robot.lib.amps
import frc.robot.lib.inches
import frc.robot.lib.inchesPerSecond
import frc.robot.lib.inchesPerSecondPerSecond
import frc.robot.lib.pd25
import frc.robot.lib.rotations
import frc.robot.lib.toAngle
import frc.robot.lib.toAngularAcceleration
import frc.robot.lib.toAngularVelocity
import frc.robot.subsystems.Elevator.Constants.ELEVATOR_GEAR_RATIO
import frc.robot.subsystems.Elevator.Constants.HIGH_POSITION
import frc.robot.subsystems.Elevator.Constants.HOME_POSITION
import frc.robot.subsystems.Elevator.Constants.LOW_POSITION
import frc.robot.subsystems.Elevator.Constants.MID_POSITION
import frc.robot.subsystems.Elevator.Constants.SPROCKET_RADIUS
import frc.robot.subsystems.Elevator.Constants.ZERO_SPEED
import kotlin.apply

object Elevator {

    private object Constants {

        const val ELEVATOR_GEAR_RATIO = 3.0 * 3.0 * 44.0 / 30.0

        val SPROCKET_RADIUS: Distance = pd25(18) / 2.0

        val ZERO_SPEED = (-2.0).inchesPerSecond.toAngularVelocity(SPROCKET_RADIUS)

        val HOME_POSITION = 0.5.inches.toAngle(SPROCKET_RADIUS)
        val LOW_POSITION = 10.0.inches.toAngle(SPROCKET_RADIUS)
        val MID_POSITION = 21.0.inches.toAngle(SPROCKET_RADIUS)
        val HIGH_POSITION = 28.0.inches.toAngle(SPROCKET_RADIUS)
    }

    enum class ElevatorState(val request: ControlRequest) {
        INITIAL(VelocityVoltage(ZERO_SPEED).withIgnoreSoftwareLimits(true)),
        HOME(MotionMagicVoltage(HOME_POSITION)),
        LOW(MotionMagicVoltage(LOW_POSITION)),
        MID(MotionMagicVoltage(MID_POSITION)),
        HIGH(MotionMagicVoltage(HIGH_POSITION)),
    }

    private val elevatorMotor =
        TalonFX(24).apply {
            val config =
                TalonFXConfiguration().apply {
                    Feedback.SensorToMechanismRatio = ELEVATOR_GEAR_RATIO

                    MotorOutput.Inverted = InvertedValue.Clockwise_Positive

                    CurrentLimits.apply {
                        StatorCurrentLimit = 60.0
                        StatorCurrentLimitEnable = true
                        SupplyCurrentLimit = 80.0
                        SupplyCurrentLimitEnable = true
                    }

                    SoftwareLimitSwitch.apply {
                        withForwardSoftLimitThreshold(30.inches.toAngle(SPROCKET_RADIUS))
                        ForwardSoftLimitEnable = true

                        ReverseSoftLimitThreshold = 0.0
                        ReverseSoftLimitEnable = true
                    }

                    Slot0.apply {
                        kA = 0.00771144
                        kP = 22.36
                        kV = 1.4784
                        kD = 0.0396
                        kG = 0.128
                    }

                    MotionMagic.apply {
                        withMotionMagicAcceleration(
                            5073.inchesPerSecondPerSecond.toAngularAcceleration(SPROCKET_RADIUS)
                        )
                        withMotionMagicCruiseVelocity(
                            32.inchesPerSecond.toAngularVelocity(SPROCKET_RADIUS)
                        )
                    }
                }

            configurator.apply(config)
        }

    var currentState = ElevatorState.INITIAL

    fun stateActions() {
        elevatorMotor.setControl(currentState.request)
    }

    fun stateTransitions() {
        when (currentState) {
            ElevatorState.INITIAL -> {
                if (elevatorMotor.statorCurrent.value >= 10.0.amps) {
                    elevatorMotor.setPosition(0.rotations)
                    currentState = ElevatorState.HOME
                }
            }
            ElevatorState.HOME,
            ElevatorState.LOW,
            ElevatorState.MID,
            ElevatorState.HIGH -> {
                if (OI.driveController.bButton) {
                    currentState = ElevatorState.HIGH
                } else if (OI.driveController.yButton) {
                    currentState = ElevatorState.MID
                } else if (OI.driveController.xButton) {
                    currentState = ElevatorState.LOW
                } else if (OI.driveController.aButton) {
                    currentState = ElevatorState.HOME
                }
            }
        }
    }
}
