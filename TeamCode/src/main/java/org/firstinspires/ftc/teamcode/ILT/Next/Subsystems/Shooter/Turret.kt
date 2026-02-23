package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter

import com.bylazar.telemetry.PanelsTelemetry.telemetry
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

import org.firstinspires.ftc.teamcode.ILT.Next.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentHeading
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.poseValid

import kotlin.math.*

object Turret : Subsystem {
    // ==================== HARDWARE ====================
    enum class State { IDLE, MANUAL, ODOMETRY, RESET_HEADING }
    var motor = MotorEx("turret")
    @JvmField var alliance = Alliance.RED

    var controller = controlSystem {
        posPid(0.3, 0.0, 0.03)
        basicFF(0.25, 0.0, 0.01)
    }

    var manualPower = 0.0
    var currentState = State.IDLE

    // ==================== PHYSICS VARIABLES - MEASURE THESE ====================
    // MEASURE: Motor encoder ticks per revolution (FROM MOTOR SPEC)
    val motorTicksPerRev = 537.7 // goBILDA 1172 or similar - CHECK YOUR MOTOR

    // MEASURE: Count teeth on motor gear
    @JvmField var motorGearTeeth = 20 // COUNT TEETH

    // MEASURE: Count teeth on output gear
    @JvmField var outputGearTeeth = 86 // COUNT TEETH

    // CALCULATED: Gear ratio
    val gearRatio: Double get() = outputGearTeeth.toDouble() / motorGearTeeth.toDouble()

    // CALCULATED: Degrees per tick
    val degreesPerTick: Double get() = (1.0 / motorTicksPerRev) * gearRatio * 360.0

    // CALCULATED: Radians per tick
    private val RADIANS_PER_TICK: Double
        get() = (2.0 * Math.PI) / (motorTicksPerRev * gearRatio)
    // MEASURE: Turret offset from robot center (if any)

    @JvmField var turretOffsetX = 0.0 // inches
    @JvmField var turretOffsetY = 0.0 // inches

    // ==================== FIELD CONSTANTS ====================
    const val FIELD_SIZE = 144.0 // inches

    // Goal positions
    const val GOAL_Y = 144.0
    const val RED_GOAL_X = 144.0
    const val BLUE_GOAL_X = 0.0

    val goalX: Double get() = if (alliance == Alliance.RED) RED_GOAL_X else BLUE_GOAL_X
    val goalY: Double = GOAL_Y

    // ==================== TUNABLE ====================
    @JvmField var minPower: Double = 0.15
    @JvmField var maxPower: Double = 0.75
    @JvmField var alignmentTolerance: Double = 2.0 // degrees
    @JvmField var visionGain: Double = 0.4
    @JvmField var kV: Double = 0.25

    // ==================== STATE ====================
    private val velTimer = ElapsedTime()
    private var lastRobotHeading = 0.0
    private var robotAngularVelocity = 0.0
    private var lastTargetSeenTime: Long = 0

    const val MIN_ANGLE = -3 * PI / 4
    const val MAX_ANGLE = 3 * PI / 4
    var turretYaw: Double = 0.0

    private const val RESET_TARGET_YAW = 0.0

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        velTimer.reset()
        lastTargetSeenTime = System.currentTimeMillis()
    }

    // ==================== PHYSICS HELPERS ====================
    // Get current turret angle in degrees
    val currentAngleDegrees: Double get() = ticksToDegrees(motor.currentPosition)

    // Get current turret angle in radians
    fun getYaw(): Double = normalizeAngle(motor.currentPosition * RADIANS_PER_TICK)

    // Convert encoder ticks to degrees
    fun ticksToDegrees(ticks: Double): Double = ticks * degreesPerTick

    // Convert degrees to encoder ticks
    fun degreesToTicks(degrees: Double): Int = (degrees / degreesPerTick).toInt()

    // Calculate distance to goal using odometry
    fun distanceToGoal(): Double {
        val dx = goalX - currentX
        val dy = goalY - currentY
        return sqrt(dx * dx + dy * dy)
    }

    // Calculate angle to goal (field frame)
    fun angleToGoalField(): Double {
        val dx = goalX - currentX
        val dy = goalY - currentY
        return Math.toDegrees(atan2(dy, dx))
    }

    // Calculate required turret angle (robot frame)
    fun angleToGoalTurret(): Double {
        val fieldAngle = angleToGoalField()
        val robotHeadingDeg = Math.toDegrees(currentHeading)
        var turretAngle = fieldAngle - robotHeadingDeg - 90.0 // -90 for turret offset
        return normalizeAngleDegrees(turretAngle)
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        turretYaw = getYaw()
        updateRobotVelocity()

        when (currentState) {
            State.IDLE -> motor.power = manualPower.coerceIn(-maxPower, maxPower)
            State.MANUAL -> motor.power = manualPower.coerceIn(-maxPower, maxPower)
            State.ODOMETRY -> aimWithOdometryOnly()
            State.RESET_HEADING -> {
                val currentYaw = getYaw()
                val error = normalizeAngle(RESET_TARGET_YAW - currentYaw)
                if (abs(error) < 0.3) {
                    motor.power = 0.0
                    currentState = State.IDLE
                    return
                }
                applyControl(RESET_TARGET_YAW, 0.0)
            }
        }

        // Telemetry
        telemetry.addData("Turret/Angle", "%.1f°".format(currentAngleDegrees))
        telemetry.addData("Turret/Target", "%.1f°".format(angleToGoalTurret()))
        telemetry.addData("Turret/Distance", "%.1f".format(distanceToGoal()))
    }

    // ==================== ODOMETRY AIMING ====================
    private fun updateRobotVelocity() {
        if (currentState != State.ODOMETRY && currentState != State.RESET_HEADING) {
            robotAngularVelocity = 0.0
            return
        }

        val dt = velTimer.seconds()
        if (dt < 0.02 || dt > 0.2) {
            robotAngularVelocity = 0.0
            velTimer.reset()
            return
        }

        val heading = currentHeading
        if (heading.isNaN() || heading.isInfinite() || !poseValid) {
            robotAngularVelocity = 0.0
            return
        }

        val deltaHeading = normalizeAngle(heading - lastRobotHeading)
        robotAngularVelocity = deltaHeading / dt
        lastRobotHeading = heading
        velTimer.reset()
    }

    private fun applyControl(targetYaw: Double, targetVelocity: Double = 0.0) {
        val clampedTarget = targetYaw.coerceIn(MIN_ANGLE, MAX_ANGLE)
        val currentYaw = getYaw()

        controller.goal = KineticState(clampedTarget, targetVelocity)
        var power = controller.calculate(KineticState(currentYaw, 0.0))

        val errorDeg = Math.toDegrees(abs(clampedTarget - currentYaw))
        if (errorDeg > 0.5) {
            power += (if (power >= 0) 1.0 else -1.0) * minPower
        } else {
            if (abs(targetVelocity) < 0.1) power = 0.0
        }

        motor.power = power.coerceIn(-maxPower, maxPower)
    }

    fun aimWithOdometryOnly() {
        if (!poseValid) return

        val deltaX = goalX - currentX
        val deltaY = goalY - currentY
        val fieldAngle = atan2(deltaY, deltaX)

        val robotHeading = if (abs(currentHeading) > 2.0 * PI) Math.toRadians(currentHeading) else currentHeading
        applyControl(normalizeAngle(fieldAngle - robotHeading), -robotAngularVelocity * kV)
    }

    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    fun normalizeAngleDegrees(degrees: Double): Double {
        var angle = degrees % 360
        if (angle > 180) angle -= 360
        if (angle < -180) angle += 360
        return angle
    }

    fun aimWithOdometry() { currentState = State.ODOMETRY }
    fun stop() { currentState = State.IDLE; motor.power = 0.0 }
    fun manual() { currentState = State.MANUAL }
    fun startHeadingReset() { currentState = State.RESET_HEADING; manualPower = 0.0 }
}