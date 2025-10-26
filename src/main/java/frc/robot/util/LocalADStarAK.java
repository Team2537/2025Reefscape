// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// NOTE: This file is available at
// https://gist.github.com/mjansen4857/a8024b55eb427184dbd10ae8923bd57d

public class LocalADStarAK implements Pathfinder {
    private final ADStarIO io = new ADStarIO();

    @Override
    public boolean isNewPathAvailable() {
        if (!Logger.hasReplaySource()) {
            io.updateIsNewPathAvailable();
        }

        Logger.processInputs("LocalADStarAK", io);

        return io.isNewPathAvailable;
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        if (!Logger.hasReplaySource()) {
            io.updateCurrentPathPoints(constraints, goalEndState);
        }

        Logger.processInputs("LocalADStarAK", io);

        if (io.currentPathPoints.isEmpty()) {
            return null;
        }

        return PathPlannerPath.fromPathPoints(io.currentPathPoints, constraints, goalEndState);
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
        if (!Logger.hasReplaySource()) {
            io.adStar.setStartPosition(startPosition);
        }
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        if (!Logger.hasReplaySource()) {
            io.adStar.setGoalPosition(goalPosition);
        }
    }

    @Override
    public void setDynamicObstacles(
            List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
        if (!Logger.hasReplaySource()) {
            io.adStar.setDynamicObstacles(obs, currentRobotPos);
        }
    }

    private static class ADStarIO implements LoggableInputs {
        public LocalADStar adStar = new LocalADStar();
        public boolean isNewPathAvailable = false;
        public List<PathPoint> currentPathPoints = Collections.emptyList();

        @Override
        public void toLog(LogTable table) {
            table.put("IsNewPathAvailable", isNewPathAvailable);

            double[] pointsLogged = new double[currentPathPoints.size() * 2];
            int idx = 0;
            for (PathPoint point : currentPathPoints) {
                pointsLogged[idx] = point.position.getX();
                pointsLogged[idx + 1] = point.position.getY();
                idx += 2;
            }

            table.put("CurrentPathPoints", pointsLogged);
        }

        @Override
        public void fromLog(LogTable table) {
            isNewPathAvailable = table.get("IsNewPathAvailable", false);

            double[] pointsLogged = table.get("CurrentPathPoints", new double[0]);

            List<PathPoint> pathPoints = new ArrayList<>();
            for (int i = 0; i < pointsLogged.length; i += 2) {
                pathPoints.add(new PathPoint(new Translation2d(pointsLogged[i], pointsLogged[i + 1]), null));
            }

            currentPathPoints = pathPoints;
        }

        public void updateIsNewPathAvailable() {
            isNewPathAvailable = adStar.isNewPathAvailable();
        }

        public void updateCurrentPathPoints(PathConstraints constraints, GoalEndState goalEndState) {
            PathPlannerPath currentPath = adStar.getCurrentPath(constraints, goalEndState);

            if (currentPath != null) {
                currentPathPoints = currentPath.getAllPathPoints();
            } else {
                currentPathPoints = Collections.emptyList();
            }
        }
    }
}
