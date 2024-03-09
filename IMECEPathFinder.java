import java.awt.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import java.util.List;

public class IMECEPathFinder {
	public int[][] grid;
	public int height, width;
	public int maxFlyingHeight;
	public double fuelCostPerUnit, climbingCostPerUnit;

	public IMECEPathFinder(String filename, int rows, int cols, int maxFlyingHeight, double fuelCostPerUnit, double climbingCostPerUnit) {

		grid = new int[rows][cols];
		this.height = rows;
		this.width = cols;
		this.maxFlyingHeight = maxFlyingHeight;
		this.fuelCostPerUnit = fuelCostPerUnit;
		this.climbingCostPerUnit = climbingCostPerUnit;

		// TODO: fill the grid variable using data from filename
		try {
			Scanner sc = new Scanner(new File(filename));
			for (int x = 0; x < rows; x++) {
				for (int y = 0; y < cols; y++) {
					if (sc.hasNextInt()) {
						grid[x][y] = sc.nextInt();
					}
				}
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
	}


	/**
	 * Draws the grid using the given Graphics object.
	 * Colors should be grayscale values 0-255, scaled based on min/max elevation values in the grid
	 */
	public void drawGrayscaleMap(Graphics g) {
		int minElevation = Integer.MAX_VALUE;
		int maxElevation = Integer.MIN_VALUE;
		for (int row = 0; row < height; row++) {
			for (int col = 0; col < width; col++) {
				int elevation = grid[row][col];
				minElevation = Math.min(minElevation, elevation);
				maxElevation = Math.max(maxElevation, elevation);
			}
		}

		// Draw the grid with grayscale values scaled based on min/max elevation
		for (int row = 0; row < height; row++) {
			for (int col = 0; col < width; col++) {
				int grayScaleValue = (int) ((grid[row][col] - minElevation) * 255.0 / (maxElevation - minElevation));
				g.setColor(new Color(grayScaleValue, grayScaleValue, grayScaleValue));
				// Swap row and col positions for correct drawing
				g.fillRect(col, row, 1, 1); // Use `col` for x, `row` for y
			}
		}
	}


	public void drawGrayscaleMapToFile() {
		try (FileWriter writer = new FileWriter("grayscaleMap.dat")) {
			// Find the minimum and maximum elevation values in the grid
			int minElevation = Integer.MAX_VALUE;
			int maxElevation = Integer.MIN_VALUE;
			for (int row = 0; row < height; row++) {
				for (int col = 0; col < width; col++) {
					int elevation = grid[row][col];
					minElevation = Math.min(minElevation, elevation);
					maxElevation = Math.max(maxElevation, elevation);
				}
			}

			// Write the grayscale values to the file
			for (int i = 0; i < height; i++) {
				for (int j = 0; j < width; j++) {
					int grayScaleValue = (int) ((grid[i][j] - minElevation) * 255.0 / (maxElevation - minElevation));
					writer.write(grayScaleValue + " ");
				}
				writer.write("\n");
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Get the most cost-efficient path from the source Point start to the destination Point end
	 * using Dijkstra's algorithm on pixels.
	 *
	 * @return the List of Points on the most cost-efficient path from start to end
	 */
	public List<Point> getMostEfficientPath(Point start, Point end) {
		// TODO: Your code goes here
		// TODO: Implement the Mission 0 algorithm here
		// Initialize cost and previousPoints arrays
		double[][] cost = new double[height][width];
		Point[][] previousPoints = new Point[height][width];

		// Initialize all costs to infinity
		for (int i = 0; i < height; i++) {
			Arrays.fill(cost[i], Double.POSITIVE_INFINITY);
		}
		// Set the cost of the start point to 0
		cost[start.y][start.x] = 0;

		// Create a priority queue to store the points based on their cost
		PriorityQueue<Point> queue = new PriorityQueue<>(Comparator.comparingDouble(p -> cost[p.y][p.x]));
		queue.offer(start);


		while (!queue.isEmpty()) {
			Point current = queue.poll();

			// Stop if we reach the destination
			if (current.equals(end)) {
				break;
			}

			// Get the neighbors of the current point
			List<Point> neighbors = getNeighbors(current);

			for (Point neighbor : neighbors) {
				// Calculate the cost of moving from the current point to the neighbor
				double distance = calculateDistance(current, neighbor);
				double climbingCost = calculateClimbingCost(current, neighbor);

				double newCost = cost[current.y][current.x] + distance * fuelCostPerUnit + climbingCost;

				// Update the cost and previous point if the new cost is lower
				if (newCost < cost[neighbor.y][neighbor.x]) {
					cost[neighbor.y][neighbor.x] = newCost;
					previousPoints[neighbor.y][neighbor.x] = current;

					// Add the neighbor to the queue
					queue.offer(neighbor);
				}
			}
		}

		// Reconstruct the path from the previous points
		List<Point> path = new ArrayList<>();
		Point current = end;

		while (current != null) {
			path.add(current);
			current = previousPoints[current.y][current.x];
		}

		// Reverse the path to get the correct order
		Collections.reverse(path);



		return path;
	}

	private double calculateDistance(Point start, Point end) {
		int deltaX = end.y - start.y;
		int deltaY = end.x - start.x;
		return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
	}

	private double calculateClimbingCost(Point start, Point end) {
		int startHeight = grid[start.y][start.x];

		// Check boundaries before accessing end point's elevation
		int endHeight = grid[end.y][end.x];
		int heightImpact = Math.max(0, endHeight - startHeight);
		return climbingCostPerUnit * heightImpact;


	}

	private List<Point> getNeighbors(Point point) {
		int x = point.x;
		int y = point.y;
		List<Point> neighbors = new ArrayList<>();

		// Check the neighbors in the desired order
		if (x > 0) {
			neighbors.add(new Point(x - 1, y)); // West
			if (y > 0) {
				neighbors.add(new Point(x - 1, y - 1)); // North West
			}
			if (y < height - 1) {
				neighbors.add(new Point(x - 1, y + 1)); // South West
			}
		}
		if (x < width - 1) {
			neighbors.add(new Point(x + 1, y)); // East
			if (y > 0) {
				neighbors.add(new Point(x + 1, y - 1)); // North East
			}
			if (y < height - 1) {
				neighbors.add(new Point(x + 1, y + 1)); // South East
			}
		}
		if (y > 0) {
			neighbors.add(new Point(x, y - 1)); // North
		}
		if (y < height - 1) {
			neighbors.add(new Point(x, y + 1)); // South
		}

		return neighbors;
	}



	/**
	 * Calculate the most cost-efficient path from source to destination.
	 *
	 * @return the total cost of this most cost-efficient path when traveling from source to destination
	 */
	public double getMostEfficientPathCost(List<Point> path) {


		double totalCost = 0.0;

		// TODO: Your code goes here, use the output from the getMostEfficientPath() method
		// Iterate over the path, calculating the cost between each pair of points
		for (int i = 0; i < path.size() - 1; i++) {
			Point start = path.get(i);
			Point end = path.get(i + 1);

			double distance = calculateDistance(start, end);
			double climbingCost = calculateClimbingCost(start, end);

			double segmentCost = distance * fuelCostPerUnit + climbingCost;
			totalCost += segmentCost;
		}

		return totalCost;
	}

	/**
	 * Draw the most cost-efficient path on top of the grayscale map from source to destination.
	 */
	public void drawMostEfficientPath(Graphics g, List<Point> path) {
		// TODO: Your code goes here, use the output from the getMostEfficientPath() method
		// Draw the grayscale map first
		drawGrayscaleMap(g);

		// Set the color for drawing the path
		g.setColor(Color.GREEN);

		// Draw each segment of the path
		for (int i = 0; i < path.size() - 1; i++) {
			Point current = path.get(i);
			Point next = path.get(i + 1);

			// Draw a line segment from current to next
			g.drawLine(current.x, current.y, next.x, next.y);
		}
	}

	/**
	 * Find an escape path from source towards East such that it has the lowest elevation change.
	 * Choose a forward step out of 3 possible forward locations, using greedy method described in the assignment instructions.
	 *
	 * @return the list of Points on the path
	 */
	public List<Point> getLowestElevationEscapePath(Point start) {
		List<Point> pathPointsList = new ArrayList<>();

		// TODO: Your code goes here
		// TODO: Implement the Mission 1 greedy approach here
		pathPointsList.add(start);

		int currentRow = start.y;
		int currentCol = start.x;

		while (currentCol < width - 1) {
			int nextRow = currentRow;
			int nextCol = currentCol + 1;
			int currentElevation = grid[currentRow][currentCol];

			int eastElevation = grid[currentRow][currentCol + 1];
			int northeastElevation = (currentRow > 0) ? grid[currentRow - 1][currentCol + 1] : Integer.MAX_VALUE;
			int southeastElevation = (currentRow < height - 1) ? grid[currentRow + 1][currentCol + 1] : Integer.MAX_VALUE;

			int elevationChangeEast = Math.abs(eastElevation - currentElevation);
			int elevationChangeNortheast = Math.abs(northeastElevation - currentElevation);
			int elevationChangeSoutheast = Math.abs(southeastElevation - currentElevation);

			// Add the next point to the path based on the elevation changes
			Point nextPoint;
			if (elevationChangeEast <= elevationChangeNortheast && elevationChangeEast <= elevationChangeSoutheast) {
				nextPoint = new Point(nextCol, nextRow);
			} else if (elevationChangeNortheast <= elevationChangeSoutheast) {
				nextPoint = new Point(nextCol, nextRow - 1);
			} else {
				nextPoint = new Point(nextCol, nextRow + 1);
			}

			pathPointsList.add(nextPoint);
			currentRow = nextPoint.y;
			currentCol = nextPoint.x;
		}


		return pathPointsList;
	}

	/**
	 * Calculate the escape path from source towards East such that it has the lowest elevation change.
	 *
	 * @return the total change in elevation for the entire path
	 */
	public int getLowestElevationEscapePathCost(List<Point> pathPointsList) {
		int totalChange = 0;
		// TODO: Your code goes here, use the output from the getLowestElevationEscapePath() method
		for (int i = 0; i < pathPointsList.size() - 1; i++) {
			Point current = pathPointsList.get(i);
			Point next = pathPointsList.get(i + 1);
			int elevationChange = Math.abs(grid[current.y][current.x] - grid[next.y][next.x]);
			totalChange += elevationChange;
		}

		return totalChange;
	}

	/**
	 * Draw the escape path from source towards East on top of the grayscale map such that it has the lowest elevation change.
	 */
	public void drawLowestElevationEscapePath(Graphics g, List<Point> pathPointsList) {
		// TODO: Your code goes here, use the output from the getLowestElevationEscapePath() method
		drawGrayscaleMap(g);

		// Set the color for drawing the path
		g.setColor(Color.YELLOW);

		// Draw each segment of the path
		for (int i = 0; i < pathPointsList.size() - 1; i++) {
			Point current = pathPointsList.get(i);
			Point next = pathPointsList.get(i + 1);

			// Draw a line segment from current to next
			g.drawLine(current.x, current.y, next.x, next.y);
		}
	}
}
