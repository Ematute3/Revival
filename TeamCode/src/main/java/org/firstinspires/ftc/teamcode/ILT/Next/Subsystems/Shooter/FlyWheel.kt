package org.firstinspires.ftc.teamcode.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.hardware.VoltageSensor

import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.ActiveOpMode.telemetry

import java.util.function.Supplier

@Configurable
object FlyWheel : Subsystem {
    // ==================== HARDWARE ====================
    lateinit var Fly1: MotorEx
    lateinit var Fly2: MotorEx
    private val battery: VoltageSensor by lazy { ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub") }

    // ==================== PHYSICS VARIABLES - MEASURE THESE ====================
    // MEASURE: Flywheel wheel diameter (meters)
    @JvmField var flywheelDiameter = 0.1 // meters - MEASURE WITH CALIPERS

    // CALCULATED: Radius
    val flywheelRadius: Double get() = flywheelDiameter / 2.0

    // MEASURE: Flywheel mass (kg)
    @JvmField var flywheelMass = 0.25 // kg - WEIGH ON SCALE

    // CALCULATED: Moment of inertia (for energy calculations)
    val momentOfInertia: Double get() = 0.5 * flywheelMass * flywheelRadius * flywheelRadius

    // FROM MOTOR SPEC
    val maxRpm = 6000.0 // Falcon 500 max
    val ticksPerRev = 2048 // Falcon 500 encoder

    // CALCULATED: Angular velocity from RPM
    fun rpmToAngularVelocity(rpm: Double): Double = (2.0 * Math.PI * rpm) / 60.0

    // CALCULATED: Linear velocity at wheel edge
    fun rpmToLinearVelocity(rpm: Double): Double = rpmToAngularVelocity(rpm) * flywheelRadius

    // ==================== TUNABLE COEFFICIENTS ====================
    @JvmField var ffCoefficients = BasicFeedforwardParameters(0.001, 0.005, 0.0)
    @JvmField var pidCoefficients = PIDCoefficients(0.011, 0.0, 0.01)

    private var controller: ControlSystem = buildController()
    private fun buildController(): ControlSystem = controlSystem {
        basicFF(ffCoefficients)
        velPid(pidCoefficients)
    }

    // ==================== VOLTAGE COMPENSATION ====================
    private const val V_NOMINAL = 12.0
    var voltFilt = 12.0
    private const val ALPHA_VOLT = 0.08
    @JvmField var voltageCompEnabled = true

    // ==================== STATE ====================
    var targetVelocity = 0.0

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        Fly1 = MotorEx("Fly1").floatMode()
        Fly2 = MotorEx("Fly2").floatMode()
        voltFilt = 12.0
        targetVelocity = 0.0
        controller = buildController()
    }

    // ==================== VELOCITY CONTROL ====================
    fun setVelocity(speed: Double) {
        targetVelocity = speed
        controller = buildController()
        controller.goal = KineticState(0.0, speed)
    }

    // ==================== PHYSICS HELPERS ====================
    // Get current linear velocity at wheel edge
    val currentLinearVelocity: Double get() = rpmToLinearVelocity(Fly1.velocity)

    // Get current angular velocity
    val currentAngularVelocity: Double get() = rpmToAngularVelocity(Fly1.velocity)

    // Calculate kinetic energy of flywheel
    val kineticEnergy: Double get() = 0.5 * momentOfInertia * currentAngularVelocity * currentAngularVelocity

    // Estimate velocity drop after ball launch (simplified)
    fun estimateVelocityDrop(ballMass: Double = 0.1): Double {
        val ballVelocity = currentLinearVelocity
        val momentum = ballMass * ballVelocity
        return momentum / momentOfInertia
    }

    // ==================== PRESETS ====================
    val off = InstantCommand { setVelocity(0.0) }
    val close = InstantCommand { setVelocity(1000.0) }
    val mid = InstantCommand { setVelocity(1250.0) }
    val far = InstantCommand { setVelocity(1500.0) }
    val max = InstantCommand { setVelocity(1500.0) }
    val maxFar = InstantCommand { setVelocity(1600.0) }
    val idle = InstantCommand { setVelocity(-300.0) }
    val runHigh = InstantCommand { setVelocity(2000.0) }

    // ==================== MOTOR CONTROL ====================
    private fun setMotorPowers(power: Double) {
        val clamped = power.coerceIn(-0.85, 0.85)
        Fly1.power = clamped
        Fly2.power = clamped
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        val voltRaw = battery.voltage.coerceAtLeast(9.0)
        voltFilt += ALPHA_VOLT * (voltRaw - voltFilt)
        val voltageRatio = V_NOMINAL / voltFilt

        val rawPower = controller.calculate(Fly1.state)
        val finalPower = if (voltageCompEnabled) {
            (rawPower * voltageRatio).coerceIn(-0.85, 0.85)
        } else {
            rawPower.coerceIn(-0.85, 0.85)
        }
        setMotorPowers(finalPower)

        // Telemetry
        telemetry.addData("Flywheel/Power", "%.3f".format(finalPower))
        telemetry.addData("Flywheel/Target Vel", "%.1f".format(targetVelocity))
        telemetry.addData("Flywheel/Actual Vel", "%.1f".format(Fly1.velocity))
        telemetry.addData("Flywheel/Vel Error", "%.1f".format(targetVelocity - Fly1.velocity))
        telemetry.addData("Flywheel/At Target", isAtTarget())

        // Physics telemetry
        telemetry.addData("Flywheel/Linear Vel", "%.2f m/s".format(currentLinearVelocity))
        telemetry.addData("Flywheel/Kinetic Energy", "%.2f J".format(kineticEnergy))
    }

    // ==================== COMMANDS ====================
    class Manual(private val shooterPower: Supplier<Double>) : Command() {
        override val isDone = false
        init { requires(FlyWheel) }
        override fun update() {
            val voltageRatio = if (FlyWheel.voltageCompEnabled) V_NOMINAL / FlyWheel.voltFilt else 1.0
            val compensated = (shooterPower.get() * voltageRatio).coerceIn(-0.85, 0.85)
            FlyWheel.setMotorPowers(compensated)
        }
    }

    // ==================== STATUS ====================
    fun isAtTarget(): Boolean = Fly1.velocity > (targetVelocity - 20.0) && Fly1.velocity < (targetVelocity + 40.0)
    fun getVelocity(): Double = Fly1.velocity
}