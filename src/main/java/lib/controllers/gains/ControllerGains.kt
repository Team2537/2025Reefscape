package lib.controllers.gains

data class ControllerGains(
    val pid: PIDGains,
    val feedforward: FeedforwardGains
)
