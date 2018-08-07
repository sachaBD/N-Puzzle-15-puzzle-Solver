import sun.reflect.generics.tree.Tree;

import java.util.*;

/**
 * Created by Sacha on 6/08/2018.
 */
public class Eight_Puzzle {
    public static void main(String[] args) {
        int[][] board = {{9, 1, 7, 3}, {2, 0, 5, 4}, {10, 11, 12, 8}, {13, 14, 6, 15}};
        board = new int[][]{{7, 2, 4}, {5, 0, 6}, {8, 3, 1}};
        Eight_Puzzle puzzle = new Eight_Puzzle(board);

        puzzle.solve(false);
    }

    public int[][] currentState;
    public static int boardHeight = 4;


    /**
     * An implementation of the Eight puzzle (more commonly known as the 15 puzzle) but extended to N pieces. Solving
     * the 15 puzzle is NP hard. A number of graph based solutions are provided below.
     *
     * @param initialState The state of the board with the inner arrays representing rows.
     */
    public Eight_Puzzle(int[][] initialState) {
        this.boardHeight = initialState.length;
        this.currentState = initialState;
    }


    /**
     * An implementation of the Eight puzzle (more commonly known as the 15 puzzle) but extended to N pieces. Solving
     * the 15 puzzle is NP hard. A number of graph based solutions are provided below.
     *
     * @param initialState The state of the board as a string with the gap represented by a '_'.
     */
    public Eight_Puzzle(String[] initialState) {
        this.boardHeight = (int) Math.sqrt(initialState.length);

        this.currentState = new int[boardHeight][boardHeight];

        for (int i = 0; i < initialState.length; i++) {
            String row = initialState[i];
            for (int j = 0; j < boardHeight; j++) {
                if (row.charAt(j) == '_') {
                    this.currentState[i][j] = 0;
                } else {
                    this.currentState[i][j] = Integer.parseInt(row.substring( j, j + 1));
                }
            }
        }

        System.out.println(Arrays.deepToString(this.currentState));
    }


    /**
     * Solve the N piece puzzle by moving it to the traditional goal state.
     *
     */
    public boolean solve(boolean printSolution) {

        int[][] goal = new int[boardHeight][boardHeight];

        for (int i = 0; i < boardHeight; i++) {
            for (int j = 0; j < boardHeight; j++) {
                goal[i][j] = i * boardHeight + j;
            }
        }

        System.out.println("Moving to goal: " + Arrays.deepToString(goal));

        // Solve using A*
        LinkedList<int[][]> result = this.AStar(this.currentState, goal);

        // Display result
        System.out.println("A* found path of length: " + result.size());
        if (printSolution) {
            for (int[][] state : result) {
                System.out.println(Arrays.deepToString(state));
            }
        }

        // Solve using BFS
        result = this.BFS(this.currentState, goal);

        // Display result
        System.out.println("BFS found path of length: " + result.size());
        if (printSolution) {
            for (int[][] state : result) {
                System.out.println(Arrays.deepToString(state));
            }
        }

        return result.size() > 0;
    }


    /**
     * This performs an A* search to the find a path from the current state to the goal state. This is not always
     * optimal but should be reasonably close to the shortest path in most cases.
     *
     * This is an A* algorithm as it do not calculate a distance between nodes, but estimates the distance from the
     * node to the goal. The distance between all states is 1, this could be changed to increase as the path gets
     * longer to encourage finding shorter paths. However, this would come at the cost of increased computation
     * required to search the additional states.
     *
     * @param currentState The starting state of the puzzle.
     * @param goalState The state to move the puzzle to.
     * @return A path that moves the puzzle from 1 state to the next.
     */
    public static LinkedList<int[][]> AStar(int[][] currentState, int[][] goalState) {
        // Stores each of the currently checked states
        // This uses a hashmap for O(1) lookup
        HashMap<String, LinkedList<int[][]>> checked = new HashMap<>();

        // This stores the states remaining to be checked.
        // A tree map is the java implementation of a SortedMap uses a Red Black tree. This allows for O(log(n))
        // lookup, addition and removal.
        // The states to be checked are sorted by their distance from the goal state (defined as the difference of
        // its entries from the goal states entries).
        TreeMap<Integer, int[][]> toCheck = new TreeMap<>();

        // Start at the starting state
        toCheck.put(1, currentState);

        // Count the number of operations
        int operations = 0;

        // Add the first state as the root node for all paths.
        LinkedList firstLL = new LinkedList<>();
        firstLL.add(currentState);
        checked.put(Arrays.deepToString(currentState), firstLL);

        // Check all possible paths
        while (toCheck.size() > 0) {
            // Change the current state to the next state, the state closest to the goal
            currentState = toCheck.remove(toCheck.firstKey());

            // Check for the goal: equality to the goalState
            if (Arrays.deepEquals(currentState, goalState)) {
                System.out.println("Found Goal!");
                System.out.println("In " + operations + " steps.");

                return checked.get(Arrays.deepToString(currentState));
            }

            // Add the surrounding states
            for (int[][] state : Eight_Puzzle.next_states(currentState, false)) {
                // Create a hashable board state
                String hashable = Arrays.deepToString(state);

                // Check if its already qued to check
                if (checked.get(hashable) == null) {
                    operations++;

                    // Mark this state as searched
                    String currentHashable = Arrays.deepToString(currentState);
                    LinkedList temp = (LinkedList) checked.get(currentHashable).clone();

                    // Store the path
                    temp.add(state);
                    checked.put(hashable, temp);

                    // Store to check
                    // Calculate the distance
                    toCheck.put(Eight_Puzzle.distance(state, goalState), state);
                }
            }
        }

        System.out.println("failed to find solution.");

        return new LinkedList<>();
    }



    /**
     * finds the optimal solution to the N piece puzzling using the Breadth First Search algorithm. This method will
     * always find the optimal solution as it searches all paths of the same length before moving on. This is also a
     * slow method.
     *
     * @param currentState The state to start from.
     * @param goalState The state to move to.
     * @return The path from the current state to the goal
     */
    public static LinkedList<int[][]> BFS(int[][] currentState, int[][] goalState) {
        // The states already checked are sorted in a HashMap for O(1) lookup
        HashMap<String, LinkedList<int[][]>> alreadyChecked = new HashMap<>();

        // The states to check are sorted as a linkedlist for O(1) insert (to the end), removal (from the end) and
        // access of the last element O(1).
        LinkedList<int[][]> toCheck = new LinkedList<>();

        // Count the number of operations
        int operations = 0;

        // Sort the starting state as the first node in all paths.
        toCheck.add(currentState);
        LinkedList firstLL = new LinkedList<>();
        firstLL.add(currentState);
        alreadyChecked.put(Arrays.deepToString(currentState), firstLL);

        while (toCheck.size() > 0) {
            // Get the new state to check
            currentState = toCheck.pop();

            // Check for the terminating condition
            // Check for the goal: equality to the goalState
            if (Arrays.deepEquals(currentState, goalState)) {
                System.out.println("Found Goal!");
                System.out.println("In " + operations + " steps.");

                return alreadyChecked.get(Arrays.deepToString(currentState));
            }

            // Check each surrounding state.
            for (int[][] state : Eight_Puzzle.next_states(currentState, false)) {

                // Check if the state is already checked
                String hashable = Arrays.deepToString(state);
                if (alreadyChecked.get(hashable) == null) {
                    operations++;

                    // Add this state to the states to check at the start
                    toCheck.addLast(state);

                    // Add this states path to the list of all paths
                    LinkedList path = (LinkedList) alreadyChecked.get(Arrays.deepToString(currentState)).clone();

                    // Add this node to the list
                    path.add(state);

                    // Save the path
                    alreadyChecked.put(hashable, path);
                }
            }

        }

        return null;
    }


    /**
     * Calculates a distance between the current n puzzle and the goal puzzle state.
     * This distance is simply the difference between all numbers on the border.
     *
     * @param state_1
     * @param state_2
     * @return
     */
    public static int distance(int[][] state_1, int[][] state_2) {
        int distance = 0;
        for (int i = 0; i < boardHeight; i++) {
            for (int j = 0; j < boardHeight; j++) {
                distance += Math.abs(state_1[i][j] - state_2[i][j]);
            }
        }

        return distance;
    }


    /**
     * Performs a Depth First Search on the n puzzle to find a path to the goal state. This does not always produce
     * the optimal path.
     *
     * @param currentState
     * @param goalState
     * @return
     */
    public static LinkedList<int[][]> DFS(int[][] currentState, int[][] goalState) {
        HashMap<String, LinkedList<int[][]>> alreadyChecked = new HashMap<>();
        LinkedList<int[][]> toCheck = new LinkedList<>();

        toCheck.add(currentState);
        LinkedList firstLL = new LinkedList<>();
        firstLL.add(currentState);
        alreadyChecked.put(Arrays.deepToString(currentState), firstLL);

        while (toCheck.size() > 0) {
            // Change the current state to the next state
            currentState = toCheck.pop();

            // Check for the goal: equality to the goalState
            if (Arrays.deepEquals(currentState, goalState)) {
                System.out.println("Found Goal!");

                alreadyChecked.get(Arrays.deepToString(currentState)).add(goalState);
                return alreadyChecked.get(Arrays.deepToString(currentState));
            }

            // Add the surrounding states
            for (int[][] state : Eight_Puzzle.next_states(currentState, false)) {
                // Create a hashable board state
                String hashable = Arrays.deepToString(state);

                // Check if its already qued to check
                if (alreadyChecked.get(hashable) == null) {
                    System.out.println(hashable);

                    // Mark this state as searched
                    String currentHashable = Arrays.deepToString(currentState);
                    LinkedList temp = (LinkedList) alreadyChecked.get(currentHashable).clone();

                    // Store the path
                    temp.add(state);
                    alreadyChecked.put(hashable, temp);

                    // Store to check
                    toCheck.addFirst(state);
                }
            }
        }

        System.out.println("failed to find solution.");

        return new LinkedList<>();
    }


    /**
     * This method recurisvely depth first searches for a path to the goal state. This typically uses the entire stack
     * due to the number of combinations of possible puzzle state. This was replaced by the above DFS method.
     *
     * @param currentState
     * @param goalState
     * @param alreadyChecked
     * @return
     */
    public static LinkedList<int[][]> rec_DFS(int[][] currentState, int[][] goalState, HashMap<String, Object>
        alreadyChecked) {
        // Check for the goal: equality to the goalState
        if (Arrays.deepEquals(currentState, goalState)) {
            System.out.println("Found Goal!");

            // Return the current state on success
            LinkedList result = new LinkedList<>();
            result.add(currentState);

            return result;
        }

        // Recursively call the next states
        for (int[][] state : Eight_Puzzle.next_states(currentState, false)) {
            // Get a hashable representation
            String hashable = Arrays.deepToString(state);

            if (alreadyChecked.get(hashable) == null) {
                System.out.println(hashable);

                // Mark this state as searched
                alreadyChecked.put(hashable, 1);
                LinkedList<int[][]> result = Eight_Puzzle.rec_DFS(state, goalState, alreadyChecked);

                // Terminates if a solution is found
                if (result.size() > 0) {
                    System.out.println("Found path...");
                    for (int[][] t : result) {
                        System.out.println((Arrays.deepToString(t)));
                    }

                    // Record this state
                    result.addLast(state);

                    return result;
                }
            }
        }

        return new LinkedList<>();
    }


    /**
     * Give an N puzzle, finds the small of the possible next states that the puzzle could move to by the player
     * swapping any of the 4 tiles around the current blank.
     *
     * @param currentState
     * @param verbose
     * @return
     */
    public static LinkedList<int[][]> next_states(int[][] currentState, boolean verbose) {
        LinkedList<int[][]> nextStates = new LinkedList<>();

        int gapRowIndex = -1;
        int gapColumnIndex = -1;
        // Find the gap
        for (int i = 0; i < boardHeight; i++) {
            for(int j = 0; j < boardHeight; j++) {
                if (currentState[i][j] == 0) {
                    gapRowIndex = i;
                    gapColumnIndex = j;
                    break;
                }
            }
        }

        // Check for a up downwards
        if (gapRowIndex > 0) {
            // There is a valid upwards move

            // Deep copy the original array
            int[][] tempBoard = new int[boardHeight][boardHeight];
            for (int i = 0; i < boardHeight; i++)
                tempBoard[i] = Arrays.copyOf(currentState[i], currentState[i].length);

            // Swap the above title
            tempBoard[gapRowIndex][gapColumnIndex] = tempBoard[gapRowIndex - 1][gapColumnIndex];

            // Place 0 in the other tile
            tempBoard[gapRowIndex - 1][gapColumnIndex] = 0;

            if (verbose) {
                System.out.println("swapped down:");
                System.out.println((Arrays.deepToString(tempBoard)));
            }

            nextStates.add(tempBoard);
        }


        // Check for a move upwards
        if (gapRowIndex < (boardHeight - 1)) {
            // There is a valid upwards move

            // Deep copy the original array
            int[][] tempBoard = new int[boardHeight][boardHeight];
            for (int i = 0; i < boardHeight; i++)
                tempBoard[i] = Arrays.copyOf(currentState[i], currentState[i].length);

            // Swap the tile below
            tempBoard[gapRowIndex][gapColumnIndex] = tempBoard[gapRowIndex + 1][gapColumnIndex];

            // Place the 0
            tempBoard[gapRowIndex + 1][gapColumnIndex] = 0;

            if (verbose) {
                System.out.println("Moved up:");
                System.out.println((Arrays.deepToString(tempBoard)));
            }

            nextStates.add(tempBoard);
        }

        // Check for a move Right
        if (gapColumnIndex > 0) {
            // There is a valid upwards move

            // Deep copy the original array
            int[][] tempBoard = new int[boardHeight][boardHeight];
            for (int i = 0; i < boardHeight; i++)
                tempBoard[i] = Arrays.copyOf(currentState[i], currentState[i].length);


            // Swap to the right
            tempBoard[gapRowIndex][gapColumnIndex] = tempBoard[gapRowIndex][gapColumnIndex - 1];

            // Place 0
            tempBoard[gapRowIndex][gapColumnIndex - 1] = 0;

            if (verbose) {
                System.out.println("Moved Left:");
                System.out.println((Arrays.deepToString(tempBoard)));
            }

            nextStates.add(tempBoard);
        }

        // Check for a move Left
        if (gapColumnIndex < (boardHeight - 1)) {
            // There is a valid upwards move

            // Deep copy the original array
            int[][] tempBoard = new int[boardHeight ][boardHeight];
            for (int i = 0; i < boardHeight; i++)
                tempBoard[i] = Arrays.copyOf(currentState[i], currentState[i].length);

            // Swap to the left
            tempBoard[gapRowIndex][gapColumnIndex] = tempBoard[gapRowIndex][gapColumnIndex + 1];

            // Place 0
            tempBoard[gapRowIndex][gapColumnIndex + 1] = 0;

            if (verbose) {
                System.out.println("Moved left:");
                System.out.println((Arrays.deepToString(tempBoard)));
            }

            nextStates.add(tempBoard);
        }

        return nextStates;
    }
}
