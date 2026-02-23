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

@Autonomous(name = "Pedro Auto 12 ball Red")
@Configurable
class PedroAutonomous : NextFTCOpMode() {

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
        follower.setStartingPose(Pose(123.0, 123.5, toRadians(37.0)))
        autoPath = AutoPath()
        alliance = Alliance.RED
        tele.run {
            addLine("Status: Initialized")
            addLine("Ready to run 3+3 autonomous")
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
        // Poses (unchanged)
        private val start = Pose(123.0, 123.5, toRadians(39.0))
        private val scorePose = Pose(96.0, 96.0, toRadians(39.0))
        private val intakePose1 = Pose(122.5, 82.5, toRadians(0.0))
        private val intakeCP1 = Pose(99.924, 81.793)
        private val intakePose2 = Pose(128.5, 56.7, Math.toRadians(0.0))
        private val intakeCP2 = Pose(83.935, 56.011)
        private val intakePose3 = Pose(128.4, 34.8, Math.toRadians(0.0))
        private val intakeCP3 = Pose(78.1, 26.0)
        private val clearBot = Pose(76.7, 13.5)
        private val sideBot = Pose(112.3, 12.5)

        val pathCount = 9

        fun next(): SequentialGroup {
            val current = index++
            return when (current) {
                0 -> SequentialGroup( // firstScore
                    InstantCommand{ FlyWheel.setVelocity(1000.0)},
                    Gate.open,
                    InstantCommand { Hood.setPosition(0.5) },
                    Delay(0.2),
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierLine(start, scorePose))
                            .setLinearHeadingInterpolation(toRadians(39.0), toRadians(39.0))
                            .build()
                    ),
                    ParallelGroup(
                        Intake.run,
                        Gate.open,
                        InstantCommand { Hood.setPosition(0.5) }
                    ),
                    Delay(1.5),
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
                            .addPath(BezierCurve(scorePose, intakeCP1, intakePose1))
                            .setConstantHeadingInterpolation(Math.toRadians(0.0))
                            .build()
                    ),
                    Delay(0.8),
                    Intake.reverse,
                    Delay(0.025),
                    Intake.stop,
                    InstantCommand { autoPath.next().schedule() }
                )

                2 -> SequentialGroup( // firstIntakeLaunch
                    ParallelGroup(
                        InstantCommand{ FlyWheel.setVelocity(1000.0)},
                        InstantCommand { Hood.setPosition(0.5) }
                    ),
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierCurve(intakePose1, scorePose))
                            .setConstantHeadingInterpolation(Math.toRadians(37.0))
                            .build()
                    ),
                    ParallelGroup(
                        InstantCommand{Gate.setPosition(0.0)},
                        Intake.run
                    ),
                    Delay(1.5),
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
                            .addPath(BezierCurve(scorePose, intakeCP2, intakePose2))
                            .setConstantHeadingInterpolation(Math.toRadians(0.0))
                            .build()
                    ),
                    Delay(0.8),
                    Intake.reverse,
                    Delay(0.025),
                    Intake.stop,
                    InstantCommand { autoPath.next().schedule() }
                )

                4 -> SequentialGroup( // secondIntakeLaunch
                    ParallelGroup(
                        InstantCommand{ FlyWheel.setVelocity(1000.0)},
                        InstantCommand { Hood.setPosition(0.5) }
                    ),
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierCurve(intakePose2, intakeCP2, scorePose))
                            .setConstantHeadingInterpolation(Math.toRadians(37.0))
                            .build()
                    ),
                    ParallelGroup(
                        InstantCommand{Gate.setPosition(0.0)},
                        Intake.run
                    ),
                    Delay(1.5),
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
                            .addPath(BezierCurve(scorePose, intakeCP3, intakePose3))
                            .setConstantHeadingInterpolation(Math.toRadians(0.0))
                            .build()
                    ),
                    Delay(0.8),
                    Intake.reverse,
                    Delay(0.00125),
                    Intake.stop,

                    InstantCommand { autoPath.next().schedule() }
                )

                6 -> SequentialGroup( // thirdIntakeLaunch
                    ParallelGroup(
                        InstantCommand{ FlyWheel.setVelocity(1000.0)},
                        InstantCommand { Hood.setPosition(0.5) }
                    ),
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierCurve(intakePose3, scorePose))
                            .setConstantHeadingInterpolation(Math.toRadians(37.0))
                            .build()
                    ),
                    ParallelGroup(
                        Gate.open,
                        Intake.run
                    ),
                    Delay(1.5),
                    ParallelGroup(
                        Gate.close,
                        Intake.stop,
                        FlyWheel.off
                    ),
                    InstantCommand { autoPath.next().schedule() }
                )

                7 -> SequentialGroup( // park: both segments chained
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierLine(scorePose, clearBot))
                            .setLinearHeadingInterpolation(toRadians(39.0), toRadians(90.0))
                            .build()
                    ),
                    FollowPath(
                        follower.pathBuilder()
                            .addPath(BezierLine(clearBot, sideBot))
                            .setLinearHeadingInterpolation(toRadians(90.0), toRadians(90.0))
                            .build()
                    )
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