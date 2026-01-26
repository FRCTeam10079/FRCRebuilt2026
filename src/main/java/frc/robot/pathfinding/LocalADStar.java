package frc.robot.pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * Implementation of AD* (Anytime Dynamic A*) running locally in a background thread.
 *
 * <p>Loads the navigation grid from the deploy directory and applies inflation for robot safety.
 *
 * <p>The algorithm runs in a background daemon thread, continuously updating the path as the start
 * or goal positions change.
 */
public class LocalADStar implements Pathfinder {

  // AD* algorithm constant (1.0 for standard A* behavior)
  private static final double EPS = 1.0;

  // Grid configuration
  private double nodeSize = PathfindingConstants.NODE_SIZE_METERS;
  private double fieldLength = PathfindingConstants.FIELD_LENGTH_METERS;
  private double fieldWidth = PathfindingConstants.FIELD_WIDTH_METERS;
  private int nodesX;
  private int nodesY;

  // AD* data structures
  private final HashMap<GridPosition, Double> g = new HashMap<>();
  private final HashMap<GridPosition, Double> rhs = new HashMap<>();
  private final HashMap<GridPosition, Pair<Double, Double>> open = new HashMap<>();
  private final HashMap<GridPosition, Pair<Double, Double>> incons = new HashMap<>();
  private final Set<GridPosition> closed = new HashSet<>();

  // Obstacle sets
  private final Set<GridPosition> obstacles = new HashSet<>();
  private final Set<GridPosition> inflatedObstacles = new HashSet<>();

  // Request state (protected by lock)
  private final ReadWriteLock requestLock = new ReentrantReadWriteLock();
  private GridPosition requestStart = new GridPosition(0, 0);
  private GridPosition requestGoal = new GridPosition(0, 0);
  private Pose2d requestRealStartPose = Pose2d.kZero;
  private Pose2d requestRealGoalPose = Pose2d.kZero;
  private Translation2d requestStartVelocity = Translation2d.kZero;
  private boolean requestMinor = false;
  private boolean requestReset = false;

  // Path output (thread-safe)
  private final AtomicReference<List<Translation2d>> currentPathWaypoints =
      new AtomicReference<>(new ArrayList<>());
  private final AtomicReference<Pose2d> currentGoalPose = new AtomicReference<>(null);
  private final AtomicBoolean newPathAvailable = new AtomicBoolean(false);
  private final AtomicBoolean isComputing = new AtomicBoolean(false);

  // Background thread
  private final Thread planningThread;
  private volatile boolean running = true;

  /** Create a new pathfinder that runs AD* locally in a background thread. */
  public LocalADStar() {
    // Load obstacles from navgrid.json
    loadObstacles("pathplanner/navgrid.json");

    // Apply inflation to obstacles for robot safety
    applyInflation();

    // Calculate grid dimensions
    nodesX = (int) Math.ceil(fieldLength / nodeSize);
    nodesY = (int) Math.ceil(fieldWidth / nodeSize);

    System.out.println("[LocalADStar] Grid: " + nodesX + "x" + nodesY + " nodes");
    System.out.println("[LocalADStar] Node size: " + nodeSize + "m");
    System.out.println("[LocalADStar] Obstacles: " + obstacles.size() + " raw, "
        + inflatedObstacles.size() + " inflated");

    // Start background planning thread
    planningThread = new Thread(this::runThread);
    planningThread.setDaemon(true);
    planningThread.setName("ADStar Planning Thread");
    planningThread.start();
  }

  /**
   * Load obstacles from a navgrid JSON file.
   *
   * @param path Path relative to deploy directory
   */
  private void loadObstacles(String path) {
    File navGridFile = new File(Filesystem.getDeployDirectory(), path);
    if (!navGridFile.exists()) {
      System.err.println("[LocalADStar] Unable to load navgrid: " + path);
      return;
    }

    try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

      // Read grid configuration
      nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();

      JSONObject fieldSize = (JSONObject) json.get("field_size");
      fieldLength = ((Number) fieldSize.get("x")).doubleValue();
      fieldWidth = ((Number) fieldSize.get("y")).doubleValue();

      // Read obstacle grid
      JSONArray grid = (JSONArray) json.get("grid");
      nodesY = grid.size();

      for (int row = 0; row < grid.size(); row++) {
        JSONArray rowArray = (JSONArray) grid.get(row);
        if (row == 0) {
          nodesX = rowArray.size();
        }
        for (int col = 0; col < rowArray.size(); col++) {
          boolean isObstacle = (boolean) rowArray.get(col);
          if (isObstacle) {
            obstacles.add(new GridPosition(col, row));
          }
        }
      }

      System.out.println("[LocalADStar] Successfully loaded navgrid: " + path);
    } catch (Exception e) {
      System.err.println("[LocalADStar] Error loading navgrid: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /** Apply inflation to all obstacles based on robot dimensions. */
  private void applyInflation() {
    int inflationCells = PathfindingConstants.INFLATION_CELLS;
    inflatedObstacles.clear();
    inflatedObstacles.addAll(obstacles);

    for (GridPosition obstacle : obstacles) {
      for (int dx = -inflationCells; dx <= inflationCells; dx++) {
        for (int dy = -inflationCells; dy <= inflationCells; dy++) {
          // Use circular inflation (Euclidean distance)
          if (Math.hypot(dx, dy) <= inflationCells) {
            inflatedObstacles.add(new GridPosition(obstacle.x() + dx, obstacle.y() + dy));
          }
        }
      }
    }
  }

  // ==================== Pathfinder Interface ====================

  @Override
  public boolean isNewPathAvailable() {
    return newPathAvailable.get();
  }

  @Override
  public List<Translation2d> getCurrentPathWaypoints() {
    newPathAvailable.set(false);
    return currentPathWaypoints.get();
  }

  @Override
  public Pose2d getGoalPose() {
    return currentGoalPose.get();
  }

  @Override
  public void setStartPose(Pose2d startPose) {
    GridPosition startPos = findClosestNonObstacle(getGridPos(startPose.getTranslation()));
    if (startPos != null && !startPos.equals(requestStart)) {
      requestLock.writeLock().lock();
      try {
        requestStart = startPos;
        requestRealStartPose = startPose;
        requestMinor = true;
        newPathAvailable.set(false);
      } finally {
        requestLock.writeLock().unlock();
      }
    }
  }

  @Override
  public void setGoalPose(Pose2d goalPose) {
    GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPose.getTranslation()));
    if (gridPos != null) {
      requestLock.writeLock().lock();
      try {
        requestGoal = gridPos;
        requestRealGoalPose = goalPose;
        requestMinor = true;
        requestReset = true;
        newPathAvailable.set(false);
        currentGoalPose.set(goalPose);
      } finally {
        requestLock.writeLock().unlock();
      }
    }
  }

  @Override
  public void setStartVelocity(Translation2d startVelocity) {
    requestLock.writeLock().lock();
    try {
      requestStartVelocity = startVelocity;
    } finally {
      requestLock.writeLock().unlock();
    }
  }

  @Override
  public void setProblem(Pose2d startPose, Pose2d goalPose, Translation2d startVelocity) {
    requestLock.writeLock().lock();
    try {
      GridPosition startPos = findClosestNonObstacle(getGridPos(startPose.getTranslation()));
      if (startPos != null) {
        requestStart = startPos;
        requestRealStartPose = startPose;
      }
      GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPose.getTranslation()));
      if (gridPos != null) {
        requestGoal = gridPos;
        requestRealGoalPose = goalPose;
        currentGoalPose.set(goalPose);
      }
      requestStartVelocity = startVelocity;
      requestMinor = true;
      requestReset = true;
      newPathAvailable.set(false);
    } finally {
      requestLock.writeLock().unlock();
    }
  }

  @Override
  public void setTeleopObstacles() {
    // Currently only using one obstacle set
    System.out.println("[LocalADStar] Using teleop obstacles");
  }

  @Override
  public void setAutoObstacles() {
    // Currently only using one obstacle set
    System.out.println("[LocalADStar] Using auto obstacles");
  }

  @Override
  public boolean isComputing() {
    return isComputing.get();
  }

  @Override
  public void shutdown() {
    running = false;
    planningThread.interrupt();
  }

  // ==================== Background Thread ====================

  @SuppressWarnings("BusyWait")
  private void runThread() {
    while (running) {
      try {
        requestLock.readLock().lock();
        boolean reset = requestReset;
        boolean minor = requestMinor;

        if (reset) {
          requestReset = false;
        }
        if (minor) {
          requestMinor = false;
        }

        if (reset || minor) {
          GridPosition start = requestStart;
          GridPosition goal = requestGoal;
          Pose2d realStart = requestRealStartPose;
          Pose2d realGoal = requestRealGoalPose;
          requestLock.readLock().unlock();

          isComputing.set(true);
          doWork(reset, minor, start, goal, realStart, realGoal);
          isComputing.set(false);
        } else {
          requestLock.readLock().unlock();
          Thread.sleep(10);
        }
      } catch (InterruptedException e) {
        // Thread interrupted, check running flag
      } catch (Exception e) {
        System.err.println("[LocalADStar] Error in planning thread: " + e.getMessage());
        e.printStackTrace();
        // Reset and continue
        requestLock.writeLock().lock();
        try {
          requestReset = true;
        } finally {
          requestLock.writeLock().unlock();
        }
      }
    }
  }

  private void doWork(
      boolean needsReset,
      boolean doMinor,
      GridPosition sStart,
      GridPosition sGoal,
      Pose2d realStartPose,
      Pose2d realGoalPose) {

    if (needsReset) {
      reset(sStart, sGoal);
    }

    if (doMinor) {
      computeOrImprovePath(sStart, sGoal);

      List<GridPosition> pathPositions = extractPath(sStart, sGoal);
      List<Translation2d> waypoints =
          convertToWaypoints(pathPositions, realStartPose, realGoalPose);

      currentPathWaypoints.set(waypoints);
      newPathAvailable.set(true);
    }
  }

  // ==================== AD* Algorithm ====================

  private void reset(GridPosition sStart, GridPosition sGoal) {
    g.clear();
    rhs.clear();
    open.clear();
    incons.clear();
    closed.clear();

    // Initialize all nodes with infinite cost
    for (int x = 0; x < nodesX; x++) {
      for (int y = 0; y < nodesY; y++) {
        GridPosition pos = new GridPosition(x, y);
        g.put(pos, Double.POSITIVE_INFINITY);
        rhs.put(pos, Double.POSITIVE_INFINITY);
      }
    }

    // Goal has zero cost-to-go
    rhs.put(sGoal, 0.0);
    open.put(sGoal, key(sGoal, sStart));
  }

  private void computeOrImprovePath(GridPosition sStart, GridPosition sGoal) {
    int iterations = 0;
    int maxIterations = nodesX * nodesY * 2; // Safety limit

    while (iterations++ < maxIterations) {
      var sv = topKey();
      if (sv == null) {
        break;
      }

      GridPosition s = sv.getFirst();
      Pair<Double, Double> v = sv.getSecond();

      if (comparePair(v, key(sStart, sStart)) >= 0
          && rhs.getOrDefault(sStart, Double.POSITIVE_INFINITY)
              .equals(g.getOrDefault(sStart, Double.POSITIVE_INFINITY))) {
        break;
      }

      open.remove(s);

      if (g.getOrDefault(s, Double.POSITIVE_INFINITY)
          > rhs.getOrDefault(s, Double.POSITIVE_INFINITY)) {
        g.put(s, rhs.get(s));
        closed.add(s);

        for (GridPosition sn : getOpenNeighbors(s)) {
          updateState(sn, sStart, sGoal);
        }
      } else {
        g.put(s, Double.POSITIVE_INFINITY);
        for (GridPosition sn : getOpenNeighbors(s)) {
          updateState(sn, sStart, sGoal);
        }
        updateState(s, sStart, sGoal);
      }
    }
  }

  private void updateState(GridPosition s, GridPosition sStart, GridPosition sGoal) {
    if (!s.equals(sGoal)) {
      double minRhs = Double.POSITIVE_INFINITY;
      for (GridPosition x : getOpenNeighbors(s)) {
        double cost = g.getOrDefault(x, Double.POSITIVE_INFINITY) + cost(s, x);
        if (cost < minRhs) {
          minRhs = cost;
        }
      }
      rhs.put(s, minRhs);
    }

    open.remove(s);

    if (!g.getOrDefault(s, Double.POSITIVE_INFINITY)
        .equals(rhs.getOrDefault(s, Double.POSITIVE_INFINITY))) {
      if (!closed.contains(s)) {
        open.put(s, key(s, sStart));
      } else {
        incons.put(s, Pair.of(0.0, 0.0));
      }
    }
  }

  private List<GridPosition> extractPath(GridPosition sStart, GridPosition sGoal) {
    if (sGoal.equals(sStart)) {
      return List.of(sGoal);
    }

    List<GridPosition> path = new ArrayList<>();
    path.add(sStart);

    GridPosition s = sStart;
    for (int k = 0; k < 400; k++) {
      GridPosition best = null;
      double bestCost = Double.POSITIVE_INFINITY;

      for (GridPosition x : getOpenNeighbors(s)) {
        double cost = g.getOrDefault(x, Double.POSITIVE_INFINITY);
        if (cost < bestCost) {
          bestCost = cost;
          best = x;
        }
      }

      if (best == null) {
        break;
      }

      s = best;
      path.add(s);

      if (s.equals(sGoal)) {
        break;
      }
    }

    return path;
  }

  private List<Translation2d> convertToWaypoints(
      List<GridPosition> path, Pose2d realStartPose, Pose2d realGoalPose) {

    if (path.isEmpty()) {
      return new ArrayList<>();
    }

    // Simplify path using line-of-sight
    List<GridPosition> simplifiedPath = simplifyPath(path);

    // Convert to field coordinates
    List<Translation2d> waypoints = new ArrayList<>();
    for (GridPosition pos : simplifiedPath) {
      waypoints.add(gridPosToTranslation(pos));
    }

    // Replace start and end with actual poses
    if (!waypoints.isEmpty()) {
      waypoints.set(0, realStartPose.getTranslation());
      waypoints.set(waypoints.size() - 1, realGoalPose.getTranslation());
    }

    return waypoints;
  }

  private List<GridPosition> simplifyPath(List<GridPosition> path) {
    if (path.size() <= 2) {
      return path;
    }

    List<GridPosition> simplified = new ArrayList<>();
    simplified.add(path.get(0));

    int current = 0;
    while (current < path.size() - 1) {
      int farthest = current + 1;

      // Find the farthest visible point
      for (int i = current + 2; i < path.size(); i++) {
        if (walkable(path.get(current), path.get(i))) {
          farthest = i;
        }
      }

      simplified.add(path.get(farthest));
      current = farthest;
    }

    return simplified;
  }

  // ==================== Helper Methods ====================

  private double cost(GridPosition s1, GridPosition s2) {
    if (isCollision(s1, s2)) {
      return Double.POSITIVE_INFINITY;
    }
    return heuristic(s1, s2);
  }

  private boolean isCollision(GridPosition s1, GridPosition s2) {
    if (inflatedObstacles.contains(s1) || inflatedObstacles.contains(s2)) {
      return true;
    }

    // Check diagonal corners
    if (s1.x() != s2.x() && s1.y() != s2.y()) {
      GridPosition corner1 = new GridPosition(s1.x(), s2.y());
      GridPosition corner2 = new GridPosition(s2.x(), s1.y());
      return inflatedObstacles.contains(corner1) || inflatedObstacles.contains(corner2);
    }

    return false;
  }

  private boolean walkable(GridPosition s1, GridPosition s2) {
    // Bresenham's line algorithm
    int x0 = s1.x();
    int y0 = s1.y();
    int x1 = s2.x();
    int y1 = s2.y();

    int dx = Math.abs(x1 - x0);
    int dy = Math.abs(y1 - y0);
    int x = x0;
    int y = y0;
    int n = 1 + dx + dy;
    int xInc = (x1 > x0) ? 1 : -1;
    int yInc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; n--) {
      if (inflatedObstacles.contains(new GridPosition(x, y))) {
        return false;
      }

      if (error > 0) {
        x += xInc;
        error -= dy;
      } else if (error < 0) {
        y += yInc;
        error += dx;
      } else {
        x += xInc;
        y += yInc;
        error -= dy;
        error += dx;
        n--;
      }
    }

    return true;
  }

  private List<GridPosition> getOpenNeighbors(GridPosition s) {
    List<GridPosition> neighbors = new ArrayList<>();

    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;

        int nx = s.x() + dx;
        int ny = s.y() + dy;

        if (nx >= 0 && nx < nodesX && ny >= 0 && ny < nodesY) {
          GridPosition neighbor = new GridPosition(nx, ny);
          if (!inflatedObstacles.contains(neighbor)) {
            neighbors.add(neighbor);
          }
        }
      }
    }

    return neighbors;
  }

  private GridPosition findClosestNonObstacle(GridPosition pos) {
    if (!inflatedObstacles.contains(pos)) {
      return pos;
    }

    // BFS to find nearest non-obstacle
    Set<GridPosition> visited = new HashSet<>();
    Queue<GridPosition> queue = new LinkedList<>();
    queue.add(pos);

    while (!queue.isEmpty()) {
      GridPosition check = queue.poll();
      if (!inflatedObstacles.contains(check)
          && check.x() >= 0
          && check.x() < nodesX
          && check.y() >= 0
          && check.y() < nodesY) {
        return check;
      }

      visited.add(check);

      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          GridPosition neighbor = new GridPosition(check.x() + dx, check.y() + dy);
          if (!visited.contains(neighbor) && !queue.contains(neighbor)) {
            queue.add(neighbor);
          }
        }
      }
    }

    return null;
  }

  private Pair<Double, Double> key(GridPosition s, GridPosition sStart) {
    double gVal = g.getOrDefault(s, Double.POSITIVE_INFINITY);
    double rhsVal = rhs.getOrDefault(s, Double.POSITIVE_INFINITY);

    if (gVal > rhsVal) {
      return Pair.of(rhsVal + EPS * heuristic(sStart, s), rhsVal);
    } else {
      return Pair.of(gVal + heuristic(sStart, s), gVal);
    }
  }

  private Pair<GridPosition, Pair<Double, Double>> topKey() {
    Map.Entry<GridPosition, Pair<Double, Double>> min = null;
    for (var entry : open.entrySet()) {
      if (min == null || comparePair(entry.getValue(), min.getValue()) < 0) {
        min = entry;
      }
    }
    return min == null ? null : Pair.of(min.getKey(), min.getValue());
  }

  private double heuristic(GridPosition s1, GridPosition s2) {
    return Math.hypot(s2.x() - s1.x(), s2.y() - s1.y());
  }

  private int comparePair(Pair<Double, Double> a, Pair<Double, Double> b) {
    int first = Double.compare(a.getFirst(), b.getFirst());
    return first != 0 ? first : Double.compare(a.getSecond(), b.getSecond());
  }

  private GridPosition getGridPos(Translation2d pos) {
    int x = (int) Math.floor(pos.getX() / nodeSize);
    int y = (int) Math.floor(pos.getY() / nodeSize);
    return new GridPosition(x, y);
  }

  private Translation2d gridPosToTranslation(GridPosition pos) {
    return new Translation2d(
        (pos.x() * nodeSize) + (nodeSize / 2.0), (pos.y() * nodeSize) + (nodeSize / 2.0));
  }
}
