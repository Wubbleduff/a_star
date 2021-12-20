This project has two components:

PathDemo - a graphical pathfinding demo that can display algorithm operations across time, making it handy for algorithm development and debugging.

Controls:
- Left click to paint walls
- Right click to erase walls
- c to clear all walls
- p to load a fixed preset wall configuration
- g to generate a random wall configuration
- h to enable/disable an A* octile heuristic
- Space to pathfind!

PathChallenge - the challenge. You are to write a function that finds the length of the SHORTEST PATH between two points on a 2D grid, as fast as possible.

path-challenge.cpp is a test harness, which will test your solution against a simple reference implementation for correctness and time your solution for you, both in absolute time and as a speedup multiplier over the reference solution.

Your implementation is to go in path-challenge-impl.cpp. It will be invoked by the test harness.

- The grid is 230 x 120, i.e. 230 columns and 120 rows.
- The start cell is at (14, 11), zero-indexed.
- The end cell is at (215, 108), zero-indexed.
- Permitted motions are horizontal/vertical and diagonal movements to immediate neighbors, with the expected linear distances of 1 and Sqrt(2), respectively.
- The borders of the grid are hard borders; no wrapping is permitted.
- Some cells are walls and cannot be traversed. The test harness randomly generates maps and will repeatedly pass them to your function as a bitarray of 120 x 230 bits, one for each cell on the grid, in row-major order, where a set bit indicates the cell is a wall, and cannot be stepped on.
- The start and end cells will never be walls.
- Return -1.0 if there is no path from the start cell to the end cell.
- The test harness will fail your solution if it differs from the reference solution by 0.0001 or more.
- You may not modify or circumvent the test harness.
- You may not use LTO, e.g. your function must not be inlined or otherwise be visible to the compiler for optimization at the call site in the test harness.
- Otherwise, pretty much anything goes environment-wise: your choice of compiler, compiler flags, external libraries, language (you could have the C++ impl be just a stub that calls your implementation of choice).

FASTEST CORRECT SOLUTION WINS!

This is intended for fun and learning; it is a challenge, not a test.

Feel free to contact me with questions / ideas / new high scores, whether via work or personal (kareem.h.omar@gmail.com, 8595531654).


Linux:
To run the demo, first install SDL2: (sudo apt install libsdl2-dev). Then the demo can be built with make, and then run it.
To run the challenge, build it with make, and then run it.

Windows:
To run the demo, I've provided a ready-made Visual Studio 2019 solution since the SDL dependency is annoying.
To run the challenge, it is simple and has no dependencies so I've just provided the raw source files. Just build them in a solution and run!
