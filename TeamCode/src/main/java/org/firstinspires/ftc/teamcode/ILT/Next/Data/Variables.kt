package org.firstinspires.ftc.teamcode.ILT.Next.Data

import com.pedropathing.math.Vector
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Turret.alliance
import org.firstinspires.ftc.teamcode.subsystem.FlyWheel
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sqrt

class Variables {
    // ==================== 1. PROJECTILE MOTION ====================
    val goalHeight = 0.984 // meters - from game manual todo
    val robotLauncherHeight = 0.238 // meters - MEASURE THIS todo
    val deltaY = goalHeight - robotLauncherHeight
    val goalX = 144.0
    var goalY = if (alliance == Alliance.RED) 144.0 else 0.0
    var robotX = PedroComponent.follower.pose.x
    var robotY = PedroComponent.follower.pose.y

    val deltaX = hypot(goalX - robotX, goalY - robotY)

    fun calculateMinVelocity(deltaY: Double, deltaX: Double, g: Double = 9.81): Double {
        return sqrt(g * (deltaY + sqrt(deltaY * deltaY + deltaX * deltaX)))
    }
    val angleToGoal = Math.toDegrees(
        atan2(goalY - robotY, goalX - robotX)
    )
    // ==================== 2. FLYWHEEL ====================
    val flywheelDiameter = 0.1 // meters - MEASURE THIS Todo
    val flywheelRadius = flywheelDiameter / 2.0
    val flywheelMass = 0.25 // kg - WEIGHT ON SCALE Todo


    val momentOfInertia = 0.5 * flywheelMass * flywheelRadius * flywheelRadius
    val currentFly1Rpm = FlyWheel.Fly1.velocity

    fun rpmToAngularVelocity(rpm: Double): Double {
        return (2.0 * Math.PI * rpm) / 60.0
    }
    val angularVelocity = rpmToAngularVelocity(currentFly1Rpm)
    val linearVelocity = angularVelocity * flywheelRadius

    // ==================== 3. INTAKE ====================
    val rollerDiameter = 0.04 // meters
    val rollerRadius = rollerDiameter / 2.0

    val wheelDiameter = 0.1 // meters
    val wheelCircumference = Math.PI * wheelDiameter
    val speedRatio = 2.5 // TUNE THROUGH TESTING

    fun calculateIntakeRpm(): Double {
        // 1. Get the vector safely (using ?. to handle nullability)
        val velocityVector = PedroComponent.follower.velocity

        // 2. Extract the magnitude (scalar speed) from the vector
        // We use ?: 0.0 to default to 0 if the follower isn't initialized yet
        val drivetrainSpeed = velocityVector?.magnitude ?: 0.0

        // 3. Now you can do the math with Double values
        val angularVelocity = (drivetrainSpeed * speedRatio) / rollerRadius

        // 4. Convert rad/s to RPM
        return (60.0 * angularVelocity) / (2.0 * Math.PI)
    }

    // ==================== 4. TURRET ====================
    val encoderTicksPerRev = 2048 // Todo gobuilda 6k
    val motorTeeth = 20 // COUNT TEETH Todo gobuilda 6k
    val outputTeeth = 86 // COUNT TEETH Todo gobuilda 6k
    val gearRatio = outputTeeth.toDouble() / motorTeeth.toDouble()
    val degreesPerTick = (1.0 / encoderTicksPerRev) * gearRatio * 360.0
    fun calculateTurretAngle(robotX: Double, robotY: Double, goalX: Double, goalY: Double): Double {
        return Math.toDegrees(atan2(goalY - robotY, goalX - robotX))
    }
}