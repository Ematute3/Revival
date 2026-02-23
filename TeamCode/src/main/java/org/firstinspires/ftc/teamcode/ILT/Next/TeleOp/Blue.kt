package org.firstinspires.ftc.teamcode.ILT.Next.TeleOp

import com.bylazar.panels.Panels
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
import dev.nextftc.hardware.driving.Drivetrain
import org.firstinspires.ftc.teamcode.ILT.Next.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Gate
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Intake


import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystem.FlyWheel
import org.firstinspires.ftc.teamcode.subsystem.FlyWheel.isAtTarget
import org.firstinspires.ftc.teamcode.subsystem.FlyWheel.motor1
import org.firstinspires.ftc.teamcode.subsystem.FlyWheel.targetVelocity
import org.firstinspires.ftc.teamcode.subsystem.FlyWheel.voltFilt
import org.firstinspires.ftc.teamcode.subsystem.FlyWheel.voltageCompEnabled
import org.firstinspires.ftc.teamcode.subsystem.Hood
import java.lang.Math.toRadians

@TeleOp(name = "Main TeleOp- BLUE", group = "Competition")
class BlueTeleOp : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry
    private val joinedTelemetry = JoinedTelemetry(telemetry, panelsTelemetry)
    private enum class AimModeTele { OFF, ODO }
    private enum class FlyModeTele{IDLE,CLOSE,FAR,MID}
    private var currentMode = AimModeTele.OFF
    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                FlyWheel, Drivetrain, Hood, Gate, Intake, Turret
            ),
            BulkReadComponent, BindingsComponent
        )
    }


    override fun onInit() {
        //follower.setStartingPose(Drive.lastKnown)
        follower.pose = Pose(72.0,72.0,0.0)
        //follower.pose = Pose(40.7, 12.5, 90.0)
        // webb blue start      follower.pose = Pose(3.0, 15.0, Math.toRadians(180.0))
    }

    override fun onStartButtonPressed() {
        PedroDriverControlled(
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false // false = field centric, true = robot centric
        ).schedule()
        currentMode = AimModeTele.ODO
        Turret.alliance = Alliance.BLUE
        bindControls()
    }

    private fun bindControls() {
        // --- DRIVER (GP1) ---
        Gamepads.gamepad1.leftTrigger.greaterThan(0.5) whenBecomesTrue(Intake.run) whenBecomesFalse(Intake.stop)
        Gamepads.gamepad1.leftBumper whenBecomesTrue(Intake.reverse) whenBecomesFalse(Intake.stop)

        Gamepads.gamepad1.rightBumper whenBecomesTrue Gate.open whenBecomesFalse Gate.close

        //clsoe shooting is 1000.0 hood all the way down
        // mid shooting is 1250 hood inbetween up and half / gotta tune at school
        // far shooting is 1500 hood up all the way/ may have to change velocity down.




        Gamepads.gamepad1.dpadUp whenBecomesTrue Hood.far
        Gamepads.gamepad1.dpadLeft whenBecomesTrue Hood.mid
        Gamepads.gamepad1.dpadDown whenBecomesTrue Hood.close


        Gamepads.gamepad1.square whenBecomesTrue { FlyWheel.setVelocity(1000.0)}
        Gamepads.gamepad1.triangle whenBecomesTrue { FlyWheel.setVelocity(1300.0)}
        Gamepads.gamepad1.cross whenBecomesTrue { FlyWheel.setVelocity(-800.0) }
        Gamepads.gamepad1.circle whenBecomesTrue { FlyWheel.setVelocity(2000.0) }

        Gamepads.gamepad2.triangle whenBecomesTrue {follower.pose =
            Pose(144.0, 0.0, 180.0)
        }
        Gamepads.gamepad2.circle whenBecomesTrue { follower.pose =
            Pose(36.0, 8.0, 90.0)}
        Gamepads.gamepad2.square whenBecomesTrue {follower.pose = Pose(22.5, 123.5, toRadians(144.0))}





    }

    override fun onUpdate() {

        PanelsTelemetry.telemetry.addData("Flywheel/Target Vel",   "%.1f".format(targetVelocity))
        PanelsTelemetry.telemetry.addData("Flywheel/Actual Vel",   "%.1f".format(motor1.velocity))
        PanelsTelemetry.telemetry.addData("Flywheel/Vel Error",    "%.1f".format(targetVelocity - motor1.velocity))
        PanelsTelemetry.telemetry.addData("Flywheel/At Target",    isAtTarget())
        PanelsTelemetry.telemetry.addData("Flywheel/Voltage",      "%.2f".format(voltFilt))
        PanelsTelemetry.telemetry.addData("Flywheel/Volt Comp On", voltageCompEnabled)
        joinedTelemetry.update()
        Drive.poseValid = true
        currentMode = AimModeTele.ODO
        Drive.currentX = PedroComponent.Companion.follower.pose.x
        Drive.currentY = PedroComponent.Companion.follower.pose.y
        Drive.currentHeading = PedroComponent.Companion.follower.pose.heading

        when (currentMode) {
            AimModeTele.OFF -> Turret.manual()
            AimModeTele.ODO -> Turret.aimWithOdometry()
        }




    }
    val reset = InstantCommand {
        ParallelGroup(
            Hood.close,
            Gate.close,
            FlyWheel.off,
            Intake.stop,

            )
    }
}