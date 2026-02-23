package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Auto

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * Shooting on the Move Subsystem
 * Calculates virtual goal position to compensate for robot movement
 *
 * Based on port stealing:
 * - Virtual goal offset for moving shots
 * - Time of flight compensation
 * - Latency compensation
 */
object ShootingOnTheMove : Subsystem {
    // ==================== CONFIGURATION ====================
    // MEASURE: Time from shoot command to ball exit (seconds)
    @JvmField var shootDelay = 0.1 // seconds - MEASURE

    // ESTIMATE: Ball time of flight (seconds) - varies with distance
    @JvmField var defaultTimeOfFlight = 0.5 // seconds - ESTIMATE

    // ==================== PHYSICS VARIABLES ====================
    // Goal positions (FROM FIELD LAYOUT)
    @JvmField var goalX = 144.0 // inches
    @JvmField var goalY = 144.0 // inches

    // CALCULATED: Virtual goal position
    var virtualGoalX = goalX
    var virtualGoalY = goalY

    // ==================== ROBOT VELOCITY ====================
    // Get from Pedro Pathing
    fun getRobotVelocity(): RobotVelocity {
        val vel = PedroComponent.Companion.follower.velocity
        return if (vel != null) {
            RobotVelocity(vel.xComponent, vel.yComponent, 0.0)
        } else {
            RobotVelocity(0.0, 0.0, 0.0)
        }
    }

    // Get robot angular velocity
    fun getRobotAngularVelocity(): Double {
        return PedroComponent.Companion.follower.angularVelocity
    }

    // ==================== TIME OF FLIGHT CALCULATION ====================
    // Estimate time for ball to reach goal
    fun estimateTimeOfFlight(distance: Double, velocity: Double): Double {
        // Simple: t = d / v
        if (velocity <= 0) return defaultTimeOfFlight
        return distance / velocity
    }

    // ==================== VIRTUAL GOAL CALCULATION ====================
    /**
     * Calculate where to aim to hit a moving target
     * P_virtual = P_goal - V_robot * (t_flight + t_delay)
     *
     * Based on port stealing formula:
     * vec P_virtual = vec P_goal - vec V_turret * (t_flight(d) + t_delay)
     */
    fun calculateVirtualGoal(
        robotX: Double,
        robotY: Double,
        robotVelX: Double,
        robotVelY: Double,
        distanceToGoal: Double,
        flywheelVelocity: Double // inches per second
    ): Pair<Double, Double> {
        // Calculate time of flight
        val timeOfFlight = estimateTimeOfFlight(distanceToGoal, flywheelVelocity)

        // Total time to compensate
        val totalTime = timeOfFlight + shootDelay

        // Calculate offset due to robot movement
        val offsetX = robotVelX * totalTime
        val offsetY = robotVelY * totalTime

        // Virtual goal = real goal - movement offset
        virtualGoalX = goalX - offsetX
        virtualGoalY = goalY - offsetY

        return Pair(virtualGoalX, virtualGoalY)
    }

    // ==================== AIM COMPENSATION ====================
    // Get adjusted turret angle when moving
    fun getCompensatedTurretAngle(
        robotX: Double,
        robotY: Double,
        robotHeading: Double
    ): Double {
        val velocity = getRobotVelocity()

        // Distance to goal
        val dx = goalX - robotX
        val dy = goalY - robotY
        val distance = sqrt(dx * dx + dy * dy)

        // Get flywheel velocity (convert from RPM if needed)
        // Assuming ~1500 RPM = ~25 inches/sec
        val flywheelVel = 25.0 * 12.0 // inches/sec (adjust for your robot)

        // Calculate virtual goal
        val (virtX, virtY) = calculateVirtualGoal(
            robotX, robotY,
            velocity.x, velocity.y,
            distance, flywheelVel
        )

        // Angle to virtual goal
        return Math.toDegrees(atan2(virtY - robotY, virtX - robotX))
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        val velocity = getRobotVelocity()

        ActiveOpMode.telemetry.addData("SOTM/Vel X", "%.2f".format(velocity.x))
        ActiveOpMode.telemetry.addData("SOTM/Vel Y", "%.2f".format(velocity.y))
        ActiveOpMode.telemetry.addData("SOTM/Virtual X", "%.1f".format(virtualGoalX))
        ActiveOpMode.telemetry.addData("SOTM/Virtual Y", "%.1f".format(virtualGoalY))
    }

    // ==================== DATA CLASS ====================
    data class RobotVelocity(
        val x: Double, // inches per second
        val y: Double,
        val angular: Double
    )
}