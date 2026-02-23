package org.firstinspires.ftc.teamcode.ILT.Next.TeleOp

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.ILT.Next.Data.Alliance

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Auto.ShootingOnTheMove
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentHeading
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.poseValid
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Gate
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Intake

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Turret.alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Vision.Limelight
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.AutoAim
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystem.FlyWheel
import java.lang.StrictMath.toRadians

/**
 * Main Competition TeleOp
 * Combines all subsystems with port stealing features:
 * - Auto-aim with distance-based RPM
 * - Shooting on the move (virtual goal)
 * - Hood compensation
 * - Limelight vision tracking
 * - Full telemetry
 */
@TeleOp(name = "Main TeleOp - Full", group = "Competition")
class MainTeleOpFull : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry
    private val joinedTelemetry = JoinedTelemetry(telemetry, panelsTelemetry)

    // ==================== ENUMS ====================
    private enum class AimModeTele {
        OFF,      // Manual turret control
        ODO,      // Odometry-based aiming
        VISION    // Limelight-assisted aiming
    }

    private enum class FlyModeTele {
        IDLE,     // Not shooting
        CLOSE,    // Close range shot
        MID,      // Mid range shot
        FAR,      // Far range shot
        AUTO      // Auto-aim with distance
    }

    private enum class ShootState {
        READY,    // Ready to shoot
        SPINNING, // Flywheel spinning up
        FIRING,   // Currently shooting
    }

    // ==================== STATE ====================
    private var currentAimMode = AimModeTele.ODO
    private var currentFlyMode = FlyModeTele.AUTO
    private var shootState = ShootState.READY
    private var autoAimEnabled = false

    // ==================== INITIALIZATION ====================
    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                FlyWheel,
                Turret,
                Hood,
                Gate,
                Intake,
                Turret,
                Limelight,
                AutoAim,
                ShootingOnTheMove,
                Drive
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        // Set starting pose based on alliance
        follower.setStartingPose(Drive.lastKnown)
        alliance = Alliance.RED
    }

    override fun onStartButtonPressed() {
        // Start drivetrain
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false  // false = field centric
        ).schedule()

        // Set default mode
        currentAimMode = AimModeTele.ODO
        autoAimEnabled = false

        bindControls()
    }

    // ==================== CONTROLS ====================
    private fun bindControls() {
        // ==================== DRIVER (GAMEPAD 1) ====================

        // --- INTAKE CONTROLS ---
        // Left Trigger: Intake
        Gamepads.gamepad1.leftTrigger.greaterThan(0.5)
            .whenBecomesTrue(Intake.run)
            .whenBecomesFalse(Intake.stop)

        // Left Bumper: Reverse/Eject
        Gamepads.gamepad1.leftBumper
            .whenBecomesTrue(Intake.reverse)
            .whenBecomesFalse(Intake.stop)

        // --- SHOOTING CONTROLS ---
        // Right Bumper: Fire (hold)
        Gamepads.gamepad1.rightBumper
            .whenBecomesTrue { fire() }
            .whenBecomesFalse {
                Gate.close
                shootState = ShootState.READY
            }

        // Right Trigger: Manual fire
        Gamepads.gamepad1.rightTrigger.greaterThan(0.5)
            .whenBecomesTrue {
                if (FlyWheel.isAtTarget()) {
                    fire()
                }
            }

        // --- FLYWHEEL PRESETS (D-Pad) ---
        // D-Pad Up: Far shot (1500 RPM)
        Gamepads.gamepad1.dpadUp
            .whenBecomesTrue {
                FlyWheel.setVelocity(1500.0)
                Hood.setAngle(45.0)
                currentFlyMode = FlyModeTele.FAR
            }

        // D-Pad Right: Mid shot (1300 RPM)
        Gamepads.gamepad1.dpadRight
            .whenBecomesTrue {
                FlyWheel.setVelocity(1300.0)
                Hood.setAngle(35.0)
                currentFlyMode = FlyModeTele.MID
            }

        // D-Pad Down: Close shot (1000 RPM)
        Gamepads.gamepad1.dpadDown
            .whenBecomesTrue {
                FlyWheel.setVelocity(1000.0)
                Hood.setAngle(25.0)
                currentFlyMode = FlyModeTele.CLOSE
            }

        // D-Pad Left: Stop flywheel
        Gamepads.gamepad1.dpadLeft
            .whenBecomesTrue {
                FlyWheel.off.schedule()
                currentFlyMode = FlyModeTele.IDLE
            }

        // --- AUTO-AIM TOGGLE ---
        // Cross (X): Toggle auto-aim
        Gamepads.gamepad1.cross
            .whenBecomesTrue {
                autoAimEnabled = !autoAimEnabled
                if (autoAimEnabled) {
                    currentAimMode = AimModeTele.ODO
                }
            }

        // --- MANUAL FLYWHEEL (BUTTONS) ---
        // Square: Close preset
        Gamepads.gamepad1.square
            .whenBecomesTrue {
                FlyWheel.setVelocity(1000.0)
                currentFlyMode = FlyModeTele.CLOSE
            }

        // Triangle: Mid preset
        Gamepads.gamepad1.triangle
            .whenBecomesTrue {
                FlyWheel.setVelocity(1300.0)
                currentFlyMode = FlyModeTele.MID
            }

        // Circle: Far preset
        Gamepads.gamepad1.circle
            .whenBecomesTrue {
                FlyWheel.setVelocity(1500.0)
                currentFlyMode = FlyModeTele.FAR
            }

        // Cross: Reverse/unjam
        Gamepads.gamepad1.cross
            .whenBecomesTrue { FlyWheel.setVelocity(-800.0) }

        // ==================== OPERATOR (GAMEPAD 2) ====================

        // --- TURRET CONTROLS ---
        // Left Stick X: Manual turret


        // Left Bumper: Toggle aim mode
        Gamepads.gamepad2.leftBumper
            .whenBecomesTrue {
                currentAimMode = when (currentAimMode) {
                    AimModeTele.OFF -> AimModeTele.ODO
                    AimModeTele.ODO -> AimModeTele.VISION
                    AimModeTele.VISION -> AimModeTele.OFF
                }
            }

        // Right Bumper: Reset turret heading
        Gamepads.gamepad2.rightBumper
            .whenBecomesTrue { Turret.startHeadingReset() }

        // --- HOOD CONTROLS ---
        // D-Pad: Manual hood positions
        Gamepads.gamepad2.dpadUp.whenBecomesTrue { Hood.setAngle(45.0) }
        Gamepads.gamepad2.dpadDown.whenBecomesTrue { Hood.setAngle(25.0) }
        Gamepads.gamepad2.dpadLeft.whenBecomesTrue { Hood.setAngle(35.0) }
        Gamepads.gamepad2.dpadRight.whenBecomesTrue { Hood.setAngle(40.0) }

        // --- PRESET POSES ---
        // Buttons: Robot poses
        Gamepads.gamepad2.triangle
            .whenBecomesTrue { follower.pose = Pose(0.0, 0.0, 0.0) }

        Gamepads.gamepad2.circle
            .whenBecomesTrue { follower.pose = Pose(108.0, 0.0, 8.0) }

        Gamepads.gamepad2.square
            .whenBecomesTrue { follower.pose = Pose(123.0, 123.5, toRadians(39.0)) }

        Gamepads.gamepad2.cross
            .whenBecomesTrue { follower.pose = Pose(50.0, 50.0, toRadians(45.0)) }

        // --- ALLIANCE SWITCH ---
        // Start button: Toggle alliance
        Gamepads.gamepad1.start
            .whenBecomesTrue {
                alliance = if (alliance == Alliance.RED) Alliance.BLUE else Alliance.RED
            }

        // Back button: Reset all
        Gamepads.gamepad1.back
            .whenBecomesTrue { resetAll() }
    }

    // ==================== FIRE SEQUENCE ====================
    private fun fire() {
        if (FlyWheel.isAtTarget()) {
            Gate.open
            shootState = ShootState.FIRING
        } else {
            shootState = ShootState.SPINNING
        }
    }

    // ==================== RESET ====================
    private fun resetAll() {
        ParallelGroup(
            Hood.down,
            Gate.close,
            FlyWheel.off,
            Intake.stop
        ).execute()

        currentFlyMode = FlyModeTele.IDLE
        autoAimEnabled = false
    }

    // ==================== MAIN LOOP ====================
    override fun onUpdate() {
        // Update pose from Pedro
        poseValid = true
        currentX = follower.pose.x
        currentY = follower.pose.y
        currentHeading = follower.pose.heading

        // ==================== AUTO-AIM ====================
        if (autoAimEnabled && currentAimMode != AimModeTele.OFF) {
            val aimParams = AutoAim.calculateAim()

            // Update turret to aim angle
            Turret.aimWithOdometry()

            // Update flywheel RPM based on distance
            FlyWheel.setVelocity(aimParams.flywheelRpm)

            // Update hood angle
            Hood.setAngle(aimParams.hoodAngle)
        }

        // ==================== AIM MODE ====================
        when (currentAimMode) {
            AimModeTele.OFF -> {
                Turret.manual()
            }
            AimModeTele.ODO -> {
                Turret.aimWithOdometry()
            }
            AimModeTele.VISION -> {
                // Vision-assisted aiming
                val mt2Pose = Limelight.getMegaTag2Pose()
                if (mt2Pose != null) {
                    Turret.aimWithOdometry()
                } else {
                    Turret.aimWithOdometry()
                }
            }
        }

        // ==================== TELEMETRY ====================
        updateTelemetry()
    }

    // ==================== TELEMETRY ====================
    private fun updateTelemetry() {
        // --- POSITION ---
        telemetry.addData("Pose/X", "%.1f".format(currentX))
        telemetry.addData("Pose/Y", "%.1f".format(currentY))
        telemetry.addData("Pose/Heading", "%.1f°".format(Math.toDegrees(currentHeading)))

        // --- AIM MODE ---
        telemetry.addData("Aim/Mode", currentAimMode)
        telemetry.addData("Aim/Auto", if (autoAimEnabled) "ON" else "OFF")

        // --- FLYWHEEL ---
        telemetry.addData("Flywheel/Target", "%.0f".format(FlyWheel.targetVelocity))
        telemetry.addData("Flywheel/Actual", "%.0f".format(FlyWheel.getVelocity()))
        telemetry.addData("Flywheel/At Target", if (FlyWheel.isAtTarget()) "YES" else "NO")
        telemetry.addData("Flywheel/Mode", currentFlyMode)

        // --- TURRET ---
        telemetry.addData("Turret/Angle", "%.1f°".format(Turret.currentAngleDegrees))
        telemetry.addData("Turret/Target", "%.1f°".format(Turret.angleToGoalTurret()))
        telemetry.addData("Turret/Distance", "%.1f\"".format(Turret.distanceToGoal()))

        // --- HOOD ---
        telemetry.addData("Hood/Angle", "%.1f°".format(Hood.currentAngle))
        telemetry.addData("Hood/Target", "%.1f°".format(Hood.targetAngle))

        // --- INTAKE ---
        telemetry.addData("Intake/State", Intake.intakeState)

        // --- SHOOTING ON THE MOVE ---
        val velocity = ShootingOnTheMove.getRobotVelocity()
        telemetry.addData("SOTM/Vel X", "%.1f".format(velocity.x))
        telemetry.addData("SOTM/Vel Y", "%.1f".format(velocity.y))
        telemetry.addData("SOTM/Virtual X", "%.1f".format(ShootingOnTheMove.virtualGoalX))
        telemetry.addData("SOTM/Virtual Y", "%.1f".format(ShootingOnTheMove.virtualGoalY))

        // --- LIMELIGHT ---
        telemetry.addData("Limelight/Tracking", if (Limelight.isTracking) "YES" else "NO")
        telemetry.addData("Limelight/MT2 X", "%.1f".format(Limelight.lastX))
        telemetry.addData("Limelight/MT2 Y", "%.1f".format(Limelight.lastY))

        // --- ALLIANCE ---
        telemetry.addData("Alliance", alliance)

        // --- AUTO AIM ---
        if (autoAimEnabled) {
            val aim = AutoAim.calculateAim()
            telemetry.addData("AutoAim/Distance", "%.2fm".format(aim.distance))
            telemetry.addData("AutoAim/RPM", "%.0f".format(aim.flywheelRpm))
            telemetry.addData("AutoAim/Hood", "%.1f°".format(aim.hoodAngle))
        }

        // Update panels telemetry
        panelsTelemetry.update()
    }
}