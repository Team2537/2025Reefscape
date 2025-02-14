package lib.math.geometry

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import lib.math.units.into
import kotlin.math.cos
import kotlin.math.sin

class Hexagon2d(
    private val center: Translation2d,
    private val radius: Distance,
    private val rotationAngle: Rotation2d = Rotation2d()
) {
    // Vertices of the hexagon, calculated lazily and rotated by the specified angle
    val vertices: List<Translation2d> by lazy {
        (0 until 6).map { i ->
            val angle = Math.toRadians(60.0 * i) // 60-degree increments
            val x = center.x + (radius into Meters) * cos(angle)
            val y = center.y + (radius into Meters) * sin(angle)
            
            // Apply rotation around the center point
            val rotatedX = center.x + (x - center.x) * cos(rotationAngle.radians) - (y - center.y) * sin(rotationAngle.radians)
            val rotatedY = center.y + (x - center.x) * sin(rotationAngle.radians) + (y - center.y) * cos(rotationAngle.radians)
            
            Translation2d(rotatedX, rotatedY)
        }
    }
    
    // Edges of the hexagon as Line2d objects
    val edges: List<Line2d> by lazy {
        vertices.zipWithNext().map { (start, end) ->
            Line2d(start, end)
        } + Line2d(vertices.last(), vertices.first()) // Close the loop
    }
    
    // Check if a point is inside the hexagon using the winding number algorithm
    fun contains(point: Translation2d): Boolean {
        var windingNumber = 0
        
        for (edge in edges) {
            val start = edge.start
            val end = edge.end
            
            if (start.y <= point.y) {
                if (end.y > point.y && isLeft(start, end, point) > 0) {
                    windingNumber++
                }
            } else {
                if (end.y <= point.y && isLeft(start, end, point) < 0) {
                    windingNumber--
                }
            }
        }
        
        return windingNumber != 0
    }
    
    // Get the closest point on the hexagon to a given point
    fun closestPoint(point: Translation2d): Translation2d {
        var closestPoint: Translation2d? = null
        var minDistance = Double.MAX_VALUE
        
        for (edge in edges) {
            val projection = edge.getClosestPoint(point)
            val distance = point.getDistance(projection)
            
            if (distance < minDistance) {
                minDistance = distance
                closestPoint = projection
            }
        }
        
        return closestPoint ?: center
    }
    
    // Helper: Check if a point is to the left, right, or on a line
    private fun isLeft(start: Translation2d, end: Translation2d, point: Translation2d): Double {
        return (end.x - start.x) * (point.y - start.y) - (point.x - start.x) * (end.y - start.y)
    }
    
    fun flip(): Hexagon2d {
        return Hexagon2d(center.flipped(), radius, rotationAngle)
    }
    
    override fun toString(): String {
        return "Hexagon2d(center=(${center.x}, ${center.y}), radius=$radius, rotationAngle=$rotationAngle)"
    }
}
