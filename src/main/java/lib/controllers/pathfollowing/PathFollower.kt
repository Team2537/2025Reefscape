package lib.controllers.pathfollowing

import choreo.trajectory.SwerveSample
import java.util.function.Consumer

interface PathFollower : Consumer<SwerveSample> {
    override fun accept(sample: SwerveSample)
}