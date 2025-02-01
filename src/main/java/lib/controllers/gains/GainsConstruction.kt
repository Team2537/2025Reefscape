package lib.controllers.gains

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController

// fake constructors

fun PIDController(gains: PIDGains): PIDController {
    return PIDController(gains.kP, gains.kI, gains.kD)
}

fun PIDController(gains: ControllerGains): PIDController {
    return PIDController(gains.pid)
}

fun ArmFeedforward(gains: FeedforwardGains, kG: Double): ArmFeedforward {
    return ArmFeedforward(gains.kS, kG, gains.kV, gains.kA)
}

fun ArmFeedforward(gains: ControllerGains, kG: Double): ArmFeedforward {
    return ArmFeedforward(gains.feedforward, kG)
}
