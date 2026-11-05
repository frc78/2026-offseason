package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import frc.robot.OI
import frc.robot.lib.amps
import frc.robot.lib.inches
import frc.robot.lib.inchesPerSecond
import frc.robot.lib.rotations
import frc.robot.lib.rotationsPerSecond
import frc.robot.lib.toAngle
import frc.robot.lib.toAngularVelocity
import frc.robot.subsystems.elevator.Constants.DEPLOYED_ROTATIONS
import frc.robot.subsystems.elevator.Constants.ELEVATOR_GEAR_RATIO
import frc.robot.subsystems.elevator.Constants.ELEVATOR_LOW_POSITION
import frc.robot.subsystems.elevator.Constants.ELEVATOR_MID_POSITION
import frc.robot.subsystems.elevator.Constants.RETRACTED_ROTATIONS
import frc.robot.subsystems.elevator.Constants.SPROCKET_RADIUS
import frc.robot.subsystems.elevator.Constants.ZERO_SPEED
import kotlin.apply

object elevator {

    private object Constants {

        const val ELEVATOR_GEAR_RATIO = 13.2
        val ELEVATOR_RETRACTED_POSITION = 0.0
        val ELEVATOR_DEPLOYED_POSITION = 29.0
        val ELEVATOR_LOW_POSITION = 10.0.inches
        val ELEVATOR_MID_POSITION = 21.0.inches
        const val SPROCKET_RADIUS = 0.659
        const val SPROCKET_CIRCUMFERENCE = (SPROCKET_RADIUS * Math.PI * 2)
        val DEPLOYED_ROTATIONS =
            (ELEVATOR_DEPLOYED_POSITION.inches / SPROCKET_CIRCUMFERENCE).magnitude().rotations
        val RETRACTED_ROTATIONS =
            (ELEVATOR_RETRACTED_POSITION.inches / SPROCKET_CIRCUMFERENCE).magnitude().rotations
        val ZERO_SPEED = (-2.0).inchesPerSecond
    }

    enum class Elevator {
        initial,
        home,
        low,
        mid,
        high,
    }

    val ZeroRequest =
        VelocityVoltage(ZERO_SPEED.toAngularVelocity(SPROCKET_RADIUS.inches))
            .withIgnoreSoftwareLimits(true)

    private val Elevator_motor =
        TalonFX(24).apply {
            val config =
                TalonFXConfiguration().apply {
                    configurator.apply() {
                        Feedback.withSensorToMechanismRatio(ELEVATOR_GEAR_RATIO)
                        CurrentLimits.apply {
                            StatorCurrentLimit = 30.0
                            SupplyCurrentLimit = 80.0
                        }
                        SoftwareLimitSwitch.apply {
                            this.ForwardSoftLimitThreshold = 95.0
                            ForwardSoftLimitEnable = true
                            ReverseSoftLimitEnable = true
                            this.ReverseSoftLimitThreshold = 0.0
                        }
                        Slot0.apply {
                            kP = 1.670
                            kV = .112
                            kD = 0.002
                            kG = 0.128
                        }
                        MotionMagic.apply {
                            MotionMagicAcceleration = 4000.0
                            MotionMagicCruiseVelocity =
                                25.inchesPerSecond
                                    .toAngularVelocity(SPROCKET_RADIUS.inches)
                                    .rotationsPerSecond
                        }
                    }
                }

        }

    var CurrentState = Elevator.initial

    fun state_actions() {
        when {
            CurrentState == Elevator.initial -> {
                Elevator_motor.setControl((ZeroRequest))
            }
            CurrentState == Elevator.home -> {
                Elevator_motor.setControl(PositionVoltage(RETRACTED_ROTATIONS))
            }
            CurrentState == Elevator.low -> {
                Elevator_motor.setControl(
                    PositionVoltage(ELEVATOR_LOW_POSITION.toAngle(SPROCKET_RADIUS.inches))
                )
            }
            CurrentState == Elevator.mid -> {
                Elevator_motor.setControl(
                    PositionVoltage(ELEVATOR_MID_POSITION.toAngle(SPROCKET_RADIUS.inches))
                )
            }
            CurrentState == Elevator.high -> {
                Elevator_motor.setControl(PositionVoltage(DEPLOYED_ROTATIONS))
            }
        }
    }

    fun state_transitions() {
        when {
            CurrentState == Elevator.initial -> {
                if (Elevator_motor.statorCurrent.value < (-10).amps) {
                    Elevator_motor.setPosition(0.rotations)
                    CurrentState = Elevator.home
                }
            }
            CurrentState == Elevator.home -> {
                if (OI.driveController.bButton) {
                    CurrentState = Elevator.high
                }
                if (OI.driveController.yButton) {
                    CurrentState = Elevator.mid
                }
                if (OI.driveController.xButton) {
                    CurrentState = Elevator.low
                }
            }
            CurrentState == Elevator.low -> {
                if (OI.driveController.bButton) {
                    CurrentState = Elevator.high
                }
                if (OI.driveController.yButton) {
                    CurrentState = Elevator.mid
                }
                if (OI.driveController.aButton) {
                    CurrentState = Elevator.home
                }
            }
            CurrentState == Elevator.mid -> {
                if (OI.driveController.bButton) {
                    CurrentState = Elevator.high
                }
                if (OI.driveController.xButton) {
                    CurrentState = Elevator.low
                }
                if (OI.driveController.aButton) {
                    CurrentState = Elevator.home
                }
            }
            CurrentState == Elevator.high -> {
                if (OI.driveController.yButton) {
                    CurrentState = Elevator.mid
                }
                if (OI.driveController.xButton) {
                    CurrentState = Elevator.low
                }
                if (OI.driveController.aButton) {
                    CurrentState = Elevator.home
                }
            }
        }
    }
}
