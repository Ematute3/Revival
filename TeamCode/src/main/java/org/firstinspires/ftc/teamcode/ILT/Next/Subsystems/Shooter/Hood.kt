package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode.telemetry

import dev.nextftc.hardware.impl.ServoEx
import kotlin.math.*

/**
 * Hood Subsystem
 * Controls shooting angle for different distances
 *
 * Based on port stealing:
 * - Dynamic hood compensation
 * - Lookup table for distance → angle
 * - Rapid fire optimization
 */
object Hood : Subsystem {
    // ==================== HARDWARE ====================
    private  var hoodServo = ServoEx("hood")

    // ==================== CONFIGURATION ====================
    // MEASURE: Hood physical limits
    @JvmField var minAngle = 20.0 // degrees - MEASURE
    @JvmField var maxAngle = 50.0 // degrees - MEASURE

    // Servo range (FROM HARDWARE)
    @JvmField var servoMinPosition = 0.0
    @JvmField var servoMaxPosition = 1.0

    // TUNE: Distance to angle lookup table (FROM TESTING)
    // Format: distance in meters → hood angle in degrees
    val distanceToAngleTable = mapOf(
        1.0 to 25.0,
        1.5 to 30.0,
        2.0 to 35.0,
        2.5 to 40.0,
        3.0 to 45.0
    )

    // ==================== STATE ====================
    var currentAngle = minAngle
    var targetAngle = minAngle

    // ==================== PHYSICS VARIABLES ====================
    // MEASURE: Goal height (FROM GAME MANUAL)
    @JvmField var goalHeight = 0.984 // meters

    // MEASURE: Robot/shooter height (FROM MEASUREMENT)
    @JvmField var shooterHeight = 0.238 // meters

    // CALCULATED: Height difference
    val heightDifference: Double get() = goalHeight - shooterHeight

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        setAngle(minAngle)
    }

    // ==================== LOOKUP TABLE ====================
    // Get hood angle from distance using interpolation
    fun getAngleForDistance(distanceMeters: Double): Double {
        val sorted = distanceToAngleTable.entries.sortedBy { it.key }

        // Out of range
        if (distanceMeters <= sorted.first().key) return sorted.first().value
        if (distanceMeters >= sorted.last().key) return sorted.last().value

        // Find bracketing points and interpolate
        for (i in 0 until sorted.size - 1) {
            val (d1, a1) = sorted[i]
            val (d2, a2) = sorted[i + 1]
            if (distanceMeters in d1..d2) {
                val t = (distanceMeters - d1) / (d2 - d1)
                return a1 + (a2 - a1) * t
            }
        }
        return sorted.last().value
    }

    // ==================== PHYSICS CALCULATION ====================
    // Calculate required launch angle using projectile motion
    // Based on: θ = arctan((v² - g*d) / (g * sqrt(d² + h²)))
    fun calculateAnglePhysics(distanceMeters: Double, velocityMps: Double): Double {
        val g = 9.81 // m/s²
        val h = heightDifference
        val d = distanceMeters
        val v = velocityMps

        // Simplified: angle to achieve required trajectory
        val v2 = v * v
        val term1 = v2 - g * d
        val term2 = g * sqrt(d * d + h * h)

        if (term2 == 0.0) return minAngle

        val angle = Math.toDegrees(atan(term1 / term2))
        return angle.coerceIn(minAngle, maxAngle)
    }

    // ==================== CONTROL ====================
    fun setAngle(angleDegrees: Double) {
        targetAngle = angleDegrees.coerceIn(minAngle, maxAngle)
        currentAngle = targetAngle

        // Map angle to servo position
        val t = (targetAngle - minAngle) / (maxAngle - minAngle)
        val servoPos = servoMinPosition + t * (servoMaxPosition - servoMinPosition)

        hoodServo.position = servoPos
    }

    // Set from distance (uses lookup table)
    fun setFromDistance(distanceMeters: Double) {
        val angle = getAngleForDistance(distanceMeters)
        setAngle(angle)
    }

    // Set from distance + velocity (uses physics)
    fun setFromDistanceAndVelocity(distanceMeters: Double, velocityMps: Double) {
        val angle = calculateAnglePhysics(distanceMeters, velocityMps)
        setAngle(angle)
    }
    fun setPosition(position1: Double) {
        hoodServo.position = position1
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        telemetry.addData("Hood/Angle", "%.1f°".format(currentAngle))
        telemetry.addData("Hood/Target", "%.1f°".format(targetAngle))
    }
}