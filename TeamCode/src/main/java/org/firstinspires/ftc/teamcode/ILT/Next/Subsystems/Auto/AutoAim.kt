package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter


import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Auto.ShootingOnTheMove
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentHeading
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.poseValid
import kotlin.math.*

/**
 * Auto-Aim Controller
 * Combines turret, hood, flywheel for unified aiming
 *
 * Based on port stealing:
 * - Unified aim command
 * - Distance-based RPM lookup
 * - Hood compensation
 * - Shooting on the move
 */
object AutoAim : Subsystem {
    // ==================== SUBSYSTEM REFERENCES ====================
    // These would be your existing subsystems
    // private val turret = Turret
    // private val flywheel = FlyWheel
    // private val hood = Hood

    // ==================== CONFIGURATION ====================
    @JvmField var enabled = true

    // Goal position (UPDATE FOR YOUR GAME)
    @JvmField var goalX = 144.0
    @JvmField var goalY = 144.0

    // ==================== SHOOTING MODES ====================
    enum class AimMode {
        STATIC,      // Stand still and shoot
        MOVING,      // Shoot while driving
        AUTO_AIM     // Full auto-aim with all compensations
    }

    var currentMode = AimMode.AUTO_AIM

    // ==================== DISTANCE → RPM LOOKUP ====================
    // Based on port stealing: y = 415.2 * ln(x) + 1198.8
    // Format: distance in meters → RPM
    val distanceToRpmTable = mapOf(
        1.0 to 1600.0,
        1.5 to 1450.0,
        2.0 to 1350.0,
        2.5 to 1280.0,
        3.0 to 1220.0
    )

    // Logarithmic fit (FROM REGRESSION)
    private const val COEFF_A = 415.2
    private const val COEFF_B = 1198.8

    // ==================== GET DISTANCE TO GOAL ====================
    fun getDistanceToGoal(robotX: Double, robotY: Double): Double {
        val dx = goalX - robotX
        val dy = goalY - robotY
        return sqrt(dx * dx + dy * dy) / 39.37 // inches to meters
    }

    // Get angle to goal
    fun getAngleToGoal(robotX: Double, robotY: Double): Double {
        return Math.toDegrees(atan2(goalY - robotY, goalX - robotX))
    }

    // ==================== RPM FROM DISTANCE ====================
    fun getRpmForDistance(distanceMeters: Double): Double {
        // Use lookup table with interpolation
        val sorted = distanceToRpmTable.entries.sortedBy { it.key }

        if (distanceMeters <= sorted.first().key) return sorted.first().value
        if (distanceMeters >= sorted.last().key) return sorted.last().value

        for (i in 0 until sorted.size - 1) {
            val (d1, rpm1) = sorted[i]
            val (d2, rpm2) = sorted[i + 1]
            if (distanceMeters in d1..d2) {
                val t = (distanceMeters - d1) / (d2 - d1)
                return rpm1 + (rpm2 - rpm1) * t
            }
        }
        return sorted.last().value
    }

    // Alternative: Logarithmic fit
    fun getRpmLogarithmic(distanceMeters: Double): Double {
        return COEFF_A * ln(distanceMeters) + COEFF_B
    }

    // ==================== FULL AUTO AIM ====================
    /**
     * Calculate all aiming parameters at once
     * Call this in your periodic to auto-aim
     */
    fun calculateAim(): AimParameters {
        if (!enabled || !poseValid) {
            return AimParameters(0.0, 0.0, 0.0, 0.0)
        }

        val rx = currentX
        val ry = currentY
        val rh = currentHeading

        // Distance and angle
        val distanceMeters = getDistanceToGoal(rx, ry)
        val angleToGoal = getAngleToGoal(rx, ry)

        // Get RPM for distance
        val targetRpm = getRpmForDistance(distanceMeters)

        // Get hood angle
        val hoodAngle = Hood.getAngleForDistance(distanceMeters)

        // Calculate compensated angle if moving
        val compensatedAngle = when (currentMode) {
            AimMode.STATIC -> angleToGoal
            AimMode.MOVING, AimMode.AUTO_AIM ->
                ShootingOnTheMove.getCompensatedTurretAngle(rx, ry, rh)
        }

        return AimParameters(
            turretAngle = compensatedAngle,
            flywheelRpm = targetRpm,
            hoodAngle = hoodAngle,
            distance = distanceMeters
        )
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        if (!enabled) return

        val params = calculateAim()

        telemetry.addData("AutoAim/Distance", "%.2fm".format(params.distance))
        telemetry.addData("AutoAim/Turret", "%.1f°".format(params.turretAngle))
        telemetry.addData("AutoAim/RPM", "%.0f".format(params.flywheelRpm))
        telemetry.addData("AutoAim/Hood", "%.1f°".format(params.hoodAngle))
        telemetry.addData("AutoAim/Mode", currentMode)
    }

    // ==================== DATA CLASS ====================
    data class AimParameters(
        val turretAngle: Double,
        val flywheelRpm: Double,
        val hoodAngle: Double,
        val distance: Double
    )
}