package lib.util

import frc.robot.RobotType
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import java.util.function.DoubleSupplier

class LoggedTunableNumber(
    private val key: String,
    private val default: Double
) : DoubleSupplier {
    
    /** The last values of the tunable numbers. */
    private val lastHasChangedValues: MutableMap<Int, Double> = HashMap()
    
    /** NT4 number for the dashboard. */
    private val dashboardNumber: LoggedNetworkNumber? = if (RobotType.isTuning) {
        LoggedNetworkNumber(key, default)
    } else {
        null
    }
    
    /** Gets the value as a double. */
    override fun getAsDouble(): Double {
        return if (RobotType.isTuning) {
            dashboardNumber!!.get()
        } else {
            default
        }
    }
    
    /**
     * Checks if the value has changed and updates the last value if it has.
     *
     * @param id The ID of the caller, used for tracking changes. Best practice is to use the hash code of the caller.
     *
     * @return Whether the value has changed.
     */
    fun hasChanged(id: Int): Boolean {
        val currentValue = getAsDouble()
        val lastValue = lastHasChangedValues[id]
        return if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues[id] = currentValue
            true
        } else {
            false
        }
    }
    
    private companion object {
        const val tableKey: String = "/Tuning"
        
        /**
         * Checks if any of the tunable numbers have changed and runs the action if they have.
         *
         *
         *
         * @param id The ID of the caller, used for tracking changes. Best practice is to use the hash code of the caller.
         * @param action The action to run if the tunable numbers have changed. List of doubles is the current values of the tunable numbers in the order they were passed.
         * @param tunableNumbers The tunable numbers to check for changes.
         */
        fun ifChanged(id: Int, action: (List<Double>) -> Unit, vararg tunableNumbers: LoggedTunableNumber) {
            if (tunableNumbers.any { it.hasChanged(id) }) {
                action(tunableNumbers.map { it.asDouble })
            }
        }
    }
}
