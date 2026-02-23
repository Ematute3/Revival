package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.extensions.pedro.PedroComponent

object Intake : Subsystem {
    // ==================== HARDWARE ====================
    private var intakeMotor = MotorEx("Intake")
    private var power = 0.0
    private var isInitialized = false

    var intakeState: IntakeState = IntakeState.STOPPED
    enum class IntakeState { STOPPED, INTAKING, EJECTING, FEEDING }

    // ==================== PHYSICS VARIABLES - MEASURE THESE ====================
    // MEASURE: Intake roller diameter (meters)
    @JvmField var rollerDiameter = 0.04 // meters - MEASURE WITH CALIPERS

    // CALCULATED: Roller radius
    val rollerRadius: Double get() = rollerDiameter / 2.0

    // MEASURE: Drive wheel diameter (meters)
    @JvmField var wheelDiameter = 0.1 // meters - MEASURE

    // CALCULATED: Wheel circumference
    val wheelCircumference: Double get() = Math.PI * wheelDiameter

    // TUNE: Speed ratio (roller speed / drivetrain speed)
    @JvmField var speedRatio = 2.5 // TUNE THROUGH TESTING

    // MEASURE: Motor encoder ticks per revolution
    val motorTicksPerRev = 537.7 // goBILDA motor - CHECK YOUR MOTOR

    // FROM MOTOR SPEC
    val maxIntakeRpm = 6000.0 // typical max

    override fun periodic() {
        intakeMotor.power = power
    }

    // ==================== PHYSICS HELPERS ====================
    // Get drivetrain speed from Pedro Pathing (m/s)
    fun getDrivetrainSpeed(): Double {
        val velocity = PedroComponent.follower.velocity
        return velocity?.magnitude ?: 0.0
    }

    // Get drivetrain RPM
    fun getDrivetrainRpm(): Double {
        val speedMps = getDrivetrainSpeed() // m/s
        val rpm = (speedMps / wheelCircumference) * 60.0
        return rpm
    }

    // Calculate required intake roller RPM
    fun calculateIntakeRpm(): Double {
        val drivetrainSpeed = getDrivetrainSpeed() // m/s

        // Angular velocity = (speed * ratio) / radius
        val angularVelocity = (drivetrainSpeed * speedRatio) / rollerRadius

        // Convert to RPM
        return (60.0 * angularVelocity) / (2.0 * Math.PI)
    }

    // Convert RPM to angular velocity (rad/s)
    fun rpmToAngularVelocity(rpm: Double): Double = (2.0 * Math.PI * rpm) / 60.0

    // Calculate roller linear speed
    fun getRollerLinearSpeed(): Double {
        val rpm = calculateIntakeRpm()
        val angularVel = rpmToAngularVelocity(rpm)
        return angularVel * rollerRadius
    }

    // ==================== COMMANDS ====================
    fun setPower(newPower: Double) { power = newPower }

    val run = InstantCommand {
        setPower(0.9)
        intakeState = IntakeState.INTAKING
    }

    val runSlow = InstantCommand { setPower(0.7) }

    val reverse = InstantCommand {
        setPower(-1.0)
        intakeState = IntakeState.EJECTING
    }

    val reverseSlow = InstantCommand {
        setPower(-0.5)
        intakeState = IntakeState.EJECTING
    }

    val feed = InstantCommand {
        setPower(1.0)
        intakeState = IntakeState.FEEDING
    }

    val stop = InstantCommand {
        setPower(0.0)
        intakeState = IntakeState.STOPPED
    }

    // ==================== STATUS ====================
    fun isRunning(): Boolean = power != 0.0

    // Telemetry
    val telemetryData: String get() = "Intake: ${intakeState}, Power: %.2f, Speed: %.1f RPM".format(
        power, calculateIntakeRpm()
    )
}