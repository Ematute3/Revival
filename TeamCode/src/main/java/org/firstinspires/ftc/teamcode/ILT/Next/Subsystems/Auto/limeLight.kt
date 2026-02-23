package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Vision

import com.bylazar.telemetry.PanelsTelemetry.telemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentHeading

/**
 * Limelight vision subsystem.
 * Handles target detection, distance calculation, and fiducial tracking.
 *
 * Based on port stealing techniques:
 * - MegaTag2 for AprilTag detection
 * - Confidence-based corrections
 * - Drift reset for odometry
 */
object Limelight : Subsystem {
    // ==================== HARDWARE ====================
    lateinit var ll: Limelight3A
    private var isInitialized = false

    val imu = IMUEx("imu", Direction.LEFT, Direction.UP)

    // ==================== CONFIGURATION ====================
    // Confidence thresholds (from port stealing)
    @JvmField var minDecisionMargin = 35.0 // MIN quality to accept
    @JvmField var maxJumpInches = 24.0 // MAX position jump to accept

    // ==================== STATE ====================
    var fiducialCount: Int = 0
        private set

    var fiducialData: String = "No fiducials"
        private set

    // Last detected pose
    var lastX = 0.0
    var lastY = 0.0
    var lastHeading = 0.0
    var isTracking = false

    // ==================== PHYSICS VARIABLES ====================
    // AprilTag positions (FROM FIELD LAYOUT - MEASURE)
    val aprilTagPositions = mapOf(
        // Example positions - UPDATE FOR YOUR GAME
        0 to Pose(0.0, 0.0, 0.0),
        1 to Pose(10.0, 10.0, 0.0),
        // Add all relevant tags
    )

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        ll = hardwareMap.get(Limelight3A::class.java, "ll")
        ll.pipelineSwitch(0)
        ll.setPollRateHz(100) // 100 Hz update rate
        ll.start()
        isInitialized = true
    }

    // ==================== GET TARGET DATA ====================
    fun getTargetX(): Double {
        val result = ll.latestResult
        return if (result != null && result.isValid) result.tx else 0.0
    }

    fun getTargetY(): Double {
        val result = ll.latestResult
        return if (result != null && result.isValid) result.ty else 0.0
    }

    fun getTargetArea(): Double {
        val result = ll.latestResult
        return if (result != null && result.isValid) result.ta else 0.0
    }

    // ==================== MEGATAG2 POSE ====================
    fun getMegaTag2Pose(): MT2Pose? {
        val result = ll.getLatestResult()
        if (result != null && result.isValid()) {
            val mt2 = result.getBotpose_MT2()
            if (mt2 != null) {
                val x = mt2.getPosition().x
                val y = mt2.getPosition().y

                // Check for impossible jumps
                val jumpDistance = if (lastX != 0.0 || lastY != 0.0) {
                    kotlin.math.hypot(x - lastX, y - lastY)
                } else 0.0

                // Only accept if within jump threshold
                if (jumpDistance < maxJumpInches || jumpDistance == 0.0) {
                    lastX = x
                    lastY = y
                    isTracking = true
                    return MT2Pose(x, y)
                }
            }
        }
        isTracking = false
        return null
    }

    // ==================== BOTPOSE ====================
    fun getBotPose(): BotPose? {
        val result = ll.getLatestResult()
        if (result != null && result.isValid()) {
            val botpose = result.botpose
            if (botpose != null) {
                val x = botpose.getPosition().x
                val y = botpose.getPosition().y
                return BotPose(x, y)
            }
        }
        return null
    }

    // ==================== UPDATE ROBOT ORIENTATION ====================
    fun updateRobotOrientation() {
        val robotYaw: Double = currentHeading
        ll.updateRobotOrientation(robotYaw)
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        updateFiducialData()
        updateLLPose()
        updateRobotOrientation()

        // Telemetry for tracking status
        telemetry.addData("Limelight/Tracking", if (isTracking) "Yes" else "No")
        telemetry.addData("Limelight/MT2 X", "%.2f".format(lastX))
        telemetry.addData("Limelight/MT2 Y", "%.2f".format(lastY))
    }

    // ==================== FIDUCIAL DATA UPDATE ====================
    fun updateFiducialData() {
        val result = ll.latestResult
        if (result != null && result.isValid) {
            val fiducials = result.fiducialResults
            fiducialCount = fiducials.size

            if (fiducials.isNotEmpty()) {
                val sb = StringBuilder()
                for (fr in fiducials) {
                    sb.append("ID: ${fr.fiducialId}, ")
                    sb.append("X: ${"%.2f".format(fr.targetXDegrees)}°, ")
                    sb.append("Strafe: ${"%.2f".format(fr.robotPoseTargetSpace.position.x)}\n")
                }
                fiducialData = sb.toString().trim()
            } else {
                fiducialData = "No fiducials detected"
            }
        } else {
            fiducialCount = 0
            fiducialData = "No valid result"
        }
    }

    // ==================== LIMELIGHT POSE UPDATE ====================
    fun updateLLPose() {
        val result = ll.getLatestResult()

        // Basic targeting data
        if (result != null && result.isValid()) {
            val tx = result.tx
            val ty = result.ty
            val ta = result.ta

            telemetry.addData("Target X", tx)
            telemetry.addData("Target Y", ty)
            telemetry.addData("Target Area", ta)
        } else {
            telemetry.addData("Limelight", "No Targets")
        }

        // BotPose (single tag)
        if (result != null && result.isValid()) {
            val botpose = result.botpose
            if (botpose != null) {
                val x = botpose.getPosition().x
                val y = botpose.getPosition().y
                telemetry.addData("MT1 Location", "($x, $y)")
            }
        }

        // MegaTag2 (multi-tag)
        if (result != null && result.isValid()) {
            val mt2 = result.getBotpose_MT2()
            if (mt2 != null) {
                val x = mt2.getPosition().x
                val y = mt2.getPosition().y
                telemetry.addData("MT2 Location:", "($x, $y)")
            }
        }
    }

    // ==================== DATA CLASSES ====================
    data class MT2Pose(val x: Double, val y: Double)
    data class BotPose(val x: Double, val y: Double)
}