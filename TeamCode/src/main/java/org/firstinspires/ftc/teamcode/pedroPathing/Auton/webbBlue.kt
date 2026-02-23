package org.firstinspires.ftc.teamcode.next.kotlin

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Gate
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Hood


import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Turret.alliance
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystem.FlyWheel

import java.lang.StrictMath.toRadians


@Autonomous(name = "webb Blue")
@Configurable
class webbBlue : NextFTCOpMode() {

    private var tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    private lateinit var autoPath: AutoPath
    private var index = 0
    private var currentMode = AimModeTele.OFF

    private enum class AimModeTele { OFF, ODO }

    init {
        addComponents(
            SubsystemComponent(Intake, FlyWheel, Gate, Hood, Drive, Turret),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        // Mirrored Y: 144 - 123.5 = 20.5 | Mirrored Heading: -37 degrees
        follower.setStartingPose(Pose(56.0, 8.0, toRadians(90.0)))
        autoPath = AutoPath()
        alliance = Alliance.BLUE // Set alliance to Blue

        tele.run {
            addLine("Status: Initialized")
            addLine("Alliance: BLUE")
            update()
        }
    }

    override fun onStartButtonPressed() {
        currentMode = AimModeTele.ODO
        autoPath.next().schedule()
        Hood.setPosition(0.0)
        Gate.setPosition(1.0)
        FlyWheel.setVelocity(500.0)
    }

    override fun onUpdate() {
        follower.update()
        when (currentMode) {
            AimModeTele.OFF -> Turret.stop()
            AimModeTele.ODO -> Turret.aimWithOdometry()
        }
        tele.run {
            addLine("Path Index: $index / ${autoPath.pathCount}")
            addLine("X: ${"%.2f".format(follower.pose.x)}")
            addLine("Y: ${"%.2f".format(follower.pose.y)}")
            addLine("Heading: ${"%.2f".format(Math.toDegrees(follower.pose.heading))}°")
            update()
        }
    }

    inner class AutoPath {
        /* Mirror logic applied:
           New Y = 144 - Old Y
           New Heading = -Old Heading
        */

        private val startPose = Pose(56.0, 8.0, Math.toRadians(90.0))
        private val intake = Pose(0.0, 15.0, Math.toRadians(180.0))
        private val scorePose = Pose(57.5, 12.0, Math.toRadians(110.0))
        val pathCount = 9

        fun next(): SequentialGroup {
            val current = index++
            return when (current) {
                0 -> SequentialGroup( // firstScore
                    InstantCommand{ FlyWheel.setVelocity(1500.0)},
                    Gate.open,
                    InstantCommand { Hood.setPosition(0.75) },
                    Delay(2.0),
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierLine(startPose, scorePose))
                            .setLinearHeadingInterpolation(toRadians(90.0), toRadians(110.0))
                            .build()
                    ),
                    ParallelGroup(
                        Intake.reverse,
                        Delay(0.05),
                        Intake.runSlow,
                        Gate.open,
                        InstantCommand { Hood.setPosition(0.75) }
                    ),
                    Delay(2.5),
                    ParallelGroup(
                        Gate.close,
                        Intake.stop
                    ),
                    InstantCommand { autoPath.next().schedule() }
                )

                1 -> SequentialGroup( // firstIntake
                    Intake.run,
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierCurve(scorePose, intake))
                            .setConstantHeadingInterpolation(Math.toRadians(180.0))
                            .build()
                    ),
                    Delay(1.5),
                    Intake.reverse,
                    Delay(0.0015),
                    Intake.stop,
                    InstantCommand { autoPath.next().schedule() }
                )

                2 -> SequentialGroup( // firstIntakeLaunch
                    ParallelGroup(
                        InstantCommand { FlyWheel.setVelocity(1500.0) },
                        InstantCommand { Hood.setPosition(0.75) }
                    ),
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierCurve(intake, scorePose))
                            .setConstantHeadingInterpolation(Math.toRadians(110.0))
                            .build()
                    ),
                    ParallelGroup(
                        Intake.reverse,
                        Delay(0.0125),
                        InstantCommand{Gate.setPosition(0.0)},
                        Intake.runSlow
                    ),
                    Delay(2.5),
                    ParallelGroup(
                        Gate.close,

                        Intake.stop
                    ),
                    InstantCommand { autoPath.next().schedule() }
                )

                3 -> SequentialGroup( // secondIntake
                    Intake.run,
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierCurve(scorePose, intake))
                            .setConstantHeadingInterpolation(Math.toRadians(180.0))
                            .build()
                    ),
                    Delay(1.5),
                    Intake.reverse,
                    Delay(0.0015),
                    Intake.stop,
                    InstantCommand { autoPath.next().schedule() }
                )

                4 -> SequentialGroup( // secondIntakeLaunch
                    ParallelGroup(
                        InstantCommand { FlyWheel.setVelocity(1500.0) },
                        InstantCommand { Hood.setPosition(0.75) }
                    ),
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierCurve(intake, scorePose))
                            .setConstantHeadingInterpolation(Math.toRadians(180.0))
                            .build()
                    ),
                    ParallelGroup(
                        Intake.reverse,
                        Delay(0.0125),
                        InstantCommand{Gate.setPosition(0.0)},
                        Intake.runSlow
                    ),
                    Delay(2.5),
                    ParallelGroup(
                        Gate.close,
                        Intake.stop
                    ),
                    InstantCommand { autoPath.next().schedule() }
                )

                5 -> SequentialGroup( // thirdIntake
                    Intake.run,
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierCurve(scorePose, intake))
                            .setConstantHeadingInterpolation(Math.toRadians(180.0))
                            .build()
                    ),
                    Delay(1.5),
                    Intake.reverse,
                    Delay(0.00125),
                    Intake.stop,

                    InstantCommand { autoPath.next().schedule() }
                )

                6 -> SequentialGroup( // thirdIntakeLaunch
                    ParallelGroup(
                        InstantCommand { FlyWheel.setVelocity(1500.0) },
                        InstantCommand { Hood.setPosition(0.75) }
                    ),
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierCurve(intake, scorePose))
                            .setConstantHeadingInterpolation(Math.toRadians(110.0))
                            .build()
                    ),
                    ParallelGroup(
                        Intake.reverse,
                        Delay(0.0125),
                        Gate.open,
                        Intake.runSlow
                    ),
                    Delay(2.5),
                    ParallelGroup(
                        Gate.close,
                        Intake.stop,
                        FlyWheel.off
                    ),
                    InstantCommand { autoPath.next().schedule() }
                )

                7 -> SequentialGroup( // thirdIntake
                    Intake.run,
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierCurve(scorePose, intake))
                            .setConstantHeadingInterpolation(Math.toRadians(180.0))
                            .build()
                    ),
                    Delay(1.5),
                    Intake.reverse,
                    Delay(0.00125),
                    Intake.stop,
                    FlyWheel.off,

                    InstantCommand { autoPath.next().schedule() }
                )

                8 -> SequentialGroup( // thirdIntakeLaunch
                    ParallelGroup(
                        InstantCommand { FlyWheel.setVelocity(1500.0) },
                        InstantCommand { Hood.setPosition(0.75) }
                    ),
                    ParallelGroup(
                        Intake.reverse,
                        Delay(0.0125),
                        Gate.open,
                        Intake.runSlow
                    ),
                    Delay(2.5),
                    ParallelGroup(
                        Gate.close,
                        Intake.stop,
                        FlyWheel.off
                    ),
                    InstantCommand { autoPath.next().schedule() }
                )

                else -> SequentialGroup()
            }
        }
    }
    override fun onStop() {
        super.onStop()
        // Save final pose so teleop can load it
        val finalPose = follower.pose
        telemetry.addData("Saving final pose", finalPose)
        telemetry.update()

        // Option A: save to static variable (simple)
        Drive.lastKnown = finalPose   // add this companion object in Drive

        // Option B: save to FTC Dashboard persistent storage or file (more reliable)
        // Use something like: GlobalStorage.savePose("lastAutoPose", finalPose)
    }
}