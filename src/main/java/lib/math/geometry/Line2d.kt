package lib.math.geometry

import edu.wpi.first.math.geometry.Translation2d

class Line2d(val start: Translation2d, val end: Translation2d) {
    
    // Compute the direction vector of the line
    private val direction: Translation2d = end.minus(start)
    
    // Calculate the length of the line
    val length: Double
        get() = start.getDistance(end)
    
    // Find the closest point on the line segment to a given point
    fun getClosestPoint(point: Translation2d): Translation2d {
        val lineVector = end.minus(start).toVector()
        val pointVector = point.minus(start).toVector()
        val lineLengthSquared = lineVector.norm() * lineVector.norm()
        
        if (lineLengthSquared == 0.0) {
            // Line is just a single point (start == end)
            return start
        }
        
        // Project the point onto the line, clamped to the segment
        val t = (pointVector.dot(lineVector) / lineLengthSquared).coerceIn(0.0, 1.0)
        return start.plus(Translation2d(lineVector.times(t)))
    }
    
    // Get the shortest distance from a point to the line segment
    fun getDistance(point: Translation2d): Double {
        val closestPoint = getClosestPoint(point)
        return point.getDistance(closestPoint)
    }
    
    // Check if a point is on the line segment (with a tolerance)
    fun isPointOnLine(point: Translation2d, tolerance: Double = 1e-6): Boolean {
        val closestPoint = getClosestPoint(point)
        return closestPoint.getDistance(point) <= tolerance
    }
    
    // Represent the line as a string
    override fun toString(): String {
        return "lib.math.geometry.Line2d(start=(${start.x}, ${start.y}), end=(${end.x}, ${end.y}))"
    }
}
