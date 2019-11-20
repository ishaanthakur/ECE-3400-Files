/* High level code - maze navigation algorithm */

#include "drivers.hpp"
#include "middleware.hpp"

// number of nodes on one side of the square. each node describes an intersection and two walls, one to its left and one to its bottom. need extra node to account for all border walls
static const uint8_t n = 10; 

// bit structure of byte that represents one node
static const uint8_t VISITED = 7; // marked visited
static const uint8_t CURRENT = 6; // marked as current position
static const uint8_t TARGET = 5; // marked as next to explore
static const uint8_t NEXT = 4; // marked as next step towards target
static const uint8_t WALLS = 2; // knowledge about the walls
static const uint8_t TEMP_WALLS = 0; // same as WALLS, except these disappear after each loop. they represent robots that must be avoided

struct position {
  /* Describes the position of an intersection */
  
  uint8_t x; // x coordinate
  uint8_t y; // y coordinate

  // initialize to (0, 0) by default
  position() {
    x = 0;
    y = 0;
  }
};

struct heading {
  /* Describes orientations */
  
  int8_t x; // x component of heading vector
  int8_t y; // y component of heading vector

  // initialize to <1, 0> by default
  heading() {
    x = 1;
    y = 0;
  }
};

struct node {
  /* Describes an intersection and two walls. Contains attributes that enable formation of graphs and fifo queues */
  
  char data; // byte that stores knowledge about intersection
  node *parent; // parent in graph
  node *fifo_next; // next in fifo queue

  node() {
    data = 0; // not visited, not current, not target, not next, and no walls by default
    parent = NULL; // no parent by default
    fifo_next = NULL; // no next node in fifo queue by default
  }
};

static node maze[n][n]; // matrix of nodes. first index is x coordinate and second index is y coordinate of the intersection
static node *robot_node = &(maze[0][0]); // current robot node, initialized to a corner of the maze
static heading robot_hdg; // current robot heading

void enqueue(node *nd, node **fifo_front, node **fifo_back) {
  /* Add a node to the fifo queue */
  
  if(*fifo_back) { // if the queue is not empty...
    (*fifo_back)->fifo_next = nd; // add the node to its back
  }
  else {
    *fifo_front = nd; // otherwise make the node its front
  }

  *fifo_back = nd; // make added node the new back
}

node* dequeue(node **fifo_front, node **fifo_back) {
  /* Remove and return the front of the fifo queue */
  
  node *ret = NULL; // return NULL by default
  
  if(*fifo_front) { // if the queue is not empty...
    ret = *fifo_front; // return node at the front
    *fifo_front = (*fifo_front)->fifo_next; // make next node the new front 
    ret->fifo_next = NULL; // sever returned node's link to the queue 
  }
  if(!*fifo_front) { // if queue now has no front...
    *fifo_back = NULL; // it should have no back either
  }

  return ret;
}

node* backtrack(node *curr, node *start) {
  /* Find next step to the target */
  
  while(curr->parent != start) { // while the next node is not the start location...
    curr = curr->parent; // go back through the graph
  }

  return curr;
}

position getPos(node *nd) {
  /* Find the position of a node's intersection */
  
  position ret;

  // 2D array elements are stored in contiguous memory locations. node position in memory determines node position in matrix
  ret.x = (nd - &(maze[0][0])) / n; 
  ret.y = (nd - &(maze[0][0])) % n;

  return ret;
}

heading calcHDG(node *source, node *target) {
  /* Compute the heading of "target" node relative to "source" node. Nodes must be adjacent */
  
  heading ret;

  if(getPos(target).x == getPos(source).x) { // if intersections are at the same x coordinate...
    ret.x = 0; // there is no heading component along x
    ret.y = 2 * (getPos(target).y > getPos(source).y) - 1; // but the y component can be 1 or -1 depending on which node has intersection at higher y
  }
  else {
    ret.x = 2 * (getPos(target).x > getPos(source).x) - 1; // otherwise the x component can be 1 or -1 depending on which node has intersection at higher x
    ret.y = 0; // and there is no component along y
  }
  
  return ret;
}

heading setHDG(heading desire) {
  /* Align the robot to a desired heading along the axes */
  
  if(robot_hdg.y == desire.x && -robot_hdg.x == desire.y) { // if it only takes one right turn to reach desire...
    turnRight(); // turning right is advantageous
  }
  else {
    while(robot_hdg.x != desire.x || robot_hdg.y != desire.y) { // otherwise, while robot is not at the desire...
      turnLeft(); // turn left

      // update the heading
      int8_t x_temp = robot_hdg.x;
      robot_hdg.x = 0 - robot_hdg.y;
      robot_hdg.y = x_temp;
    }
  }

  return desire;
}

void updateWalls() {
  /* Find and mark the walls around an intersection */ 
  
  position pos = getPos(robot_node); // get the robot position
  
  if(checkWallFront()) { // if there is a wall to the front of the robot...
    // mark the bit corresponding to the type of wall (left or bottom) in the byte of the correct node (which might not have its intersection at the current position).
    maze[pos.x + (robot_hdg.x == 1)][pos.y + (robot_hdg.y == 1)].data |= 0b1 << (WALLS + (robot_hdg.y != 0));
    Serial.println("Wall in front");
  }
  if(checkWallLeft()) { // if there is a wall to the left of the robot...
    // mark the bit corresponding to the type of wall (left or bottom) in the byte of the correct node
    maze[pos.x + (robot_hdg.y == -1)][pos.y + (robot_hdg.x == 1)].data |= 0b1 << (WALLS + (robot_hdg.x != 0));
    Serial.println("Wall to the left");
  }
  if(checkWallRight()) { // if there is a wall to the right of the robot...
    // mark the bit corresponding to the type of wall (left or bottom) in the byte of the correct node
    maze[pos.x + (robot_hdg.y == 1)][pos.y + (robot_hdg.x == -1)].data |= 0b1 << (WALLS + (robot_hdg.x != 0));
    Serial.println("Wall to the right");
  } 
}

node* greedy() {
  /* Greedy algorithm implementation. Find the closest unexplored node from the current position, taking into account all current knowledge. Return next step towards that node */
  
  node *ret = NULL;
  node *fifo_front = NULL; 
  node *fifo_back = NULL;
  
  enqueue(robot_node, &fifo_front, &fifo_back); // start a "mental" BFS from the current node to find the target

  while(fifo_front) { // while there are still frontier nodes to analyze...
    node *bfs_node = dequeue(&fifo_front, &fifo_back); // extract one from the fifo

    position bfs_pos = getPos(bfs_node); // get the position of that node's intersection
    heading bfs_hdg;

    // get the heading robot would have upon reaching that node
    if(bfs_node == robot_node) { // if it's the current robot node...
      heading bfs_hdg = robot_hdg; // the heading is the current robot heading
    }
    else {
      bfs_hdg = calcHDG(bfs_node->parent, bfs_node); // otherwise we need to compute the relative heading to its parent
    }
          
    if(!(bfs_node->data & (0b1 << VISITED))) { // if we found an unexplored node...   
      bfs_node->data |= 0b1 << TARGET; // BFS guarantees that it's the closest one. mark it as the target
      char msg[50];
      snprintf(msg, 50, "Going to (%d, %d)", bfs_pos.x, bfs_pos.y);
      Serial.println(msg);
      
      ret = backtrack(bfs_node, robot_node); // find the next step towards it

      break; // and end the search
    }

    // node is explored if we reached this point. need to branch out more

    // prefer to explore node in the front first. this results in the robot making fewer turns (which are much cheaper than moving between nodes, but still have a price) 
    node *front_node = &(maze[bfs_pos.x + bfs_hdg.x][bfs_pos.y + bfs_hdg.y]); // find the node to the front 
    if(!front_node->parent && front_node != robot_node) { // if it hasn't been added to the graph yet... 
      if(!(maze[bfs_pos.x + (bfs_hdg.x == 1)][bfs_pos.y + (bfs_hdg.y == 1)].data & (0b1 << (WALLS + (bfs_hdg.y != 0)) | 0b1 << (TEMP_WALLS + (bfs_hdg.y != 0))))) { // if accessible...
        front_node->parent = bfs_node; // link it to the node being analyzed
        enqueue(front_node, &fifo_front, &fifo_back); // put it in the fifo
      }
    }

    node *left_node = &(maze[bfs_pos.x - bfs_hdg.y][bfs_pos.y + bfs_hdg.x]); // find the node to the left
    if(!left_node->parent && left_node != robot_node) { // if it hasn't been added to the graph yet... 
      if(!(maze[bfs_pos.x + (bfs_hdg.y == -1)][bfs_pos.y + (bfs_hdg.x == 1)].data & (0b1 << (WALLS + (bfs_hdg.x != 0))))) { // if accessible (temporary walls can only exist to the front)...
        left_node->parent = bfs_node; // link it to the node being analyzed
        enqueue(left_node, &fifo_front, &fifo_back); // put it in the fifo
      }
    }
    
    node *right_node = &(maze[bfs_pos.x + bfs_hdg.y][bfs_pos.y - bfs_hdg.x]); // find the node to the right
    if(!right_node->parent && right_node != robot_node) { // if it hasn't been added to the graph yet...
      if(!(maze[bfs_pos.x + (bfs_hdg.y == 1)][bfs_pos.y + (bfs_hdg.x == -1)].data & (0b1 << (WALLS + (bfs_hdg.x != 0))))) { // if accessible...
        right_node->parent = bfs_node; // link it to the node being analyzed
        enqueue(right_node, &fifo_front, &fifo_back); // put it in the fifo
      }
    }

    node *back_node = &(maze[bfs_pos.x - bfs_hdg.x][bfs_pos.y - bfs_hdg.y]); // find the node to the back
    if(!back_node->parent && back_node != robot_node) { // if it hasn't been added to the graph yet...
      if(!(maze[bfs_pos.x + (bfs_hdg.x == -1)][bfs_pos.y + (bfs_hdg.y == -1)].data & (0b1 << (WALLS + (bfs_hdg.y != 0))))) { // if accessible...
        back_node->parent = bfs_node; // link it to the node being analyzed
        enqueue(back_node, &fifo_front, &fifo_back); // put it in the fifo
      }
    }
  }

  // if all reachable nodes explored and there is a temporary wall to the front of the robot (another robot)...
  if(!ret && (maze[getPos(robot_node).x + (robot_hdg.x == 1)][getPos(robot_node).y + (robot_hdg.y == 1)].data & (0b1 << (TEMP_WALLS + (robot_hdg.y != 0))))) {
    ret = &(maze[getPos(robot_node).x + robot_hdg.x][getPos(robot_node).y + robot_hdg.y]); // retry to go forward
    Serial.println("Blocked and all reacheable nodes explored. Attempting to clear block"); 
  }

  if(ret) { // if robot still has a next step to take...
    ret->data |= 0b1 << NEXT; // mark the corresponding bit in the next node's byte
    char msg[50];
    snprintf(msg, 50, "Next step is (%d, %d)", getPos(ret).x, getPos(ret).y);
    Serial.println(msg);
  }
  else { // otherwise robot is done and function returns NULL
    Serial.println("All nodes explored!");
  }
  
  return ret;
}

void waitForGo() {
  /* Wait for the user to send 'f' or 'F' before resuming execution */ 
  
  Serial.println("Press F to pay respects");
  char recv = 0;
  while(recv != 'f' && recv != 'F') { // while the correct character hasn't been received from serial...
    if(Serial.available()) { // if something has been received...
      recv = Serial.read(); // read it
    }
  }
}

void cleanNodes() {
  /* Remove special status and temporary walls for all nodes and destroy graph and fifo */
  
  for(uint8_t x = 0; x < n; x++) {
    for(uint8_t y = 0; y < n; y++) {
      maze[x][y].data &= ~(0b1 << CURRENT | 0b1 << TARGET | 0b1 << NEXT | 0b11 << TEMP_WALLS);
      maze[x][y].parent = NULL;
      maze[x][y].fifo_next = NULL;
    }
  }
}

void setup() {
  /* Runs once upon startup */
  
  Serial.begin(9600); // initialize serial comms

  delay(200); // wait for wall sensors to  stabilize?

  adcBeepInit(); // initialize ADC for start signal detection
  //waitForStart(); // wait for start signal

  ledInit(); // initialize indicators
  adcAnalogReadInit(); // initialize ADC for analogRead()
  servoInit(); // initialize servos
  irInit(); // initialize IR lights 

  irOn(); // turn on IR lights
  calibrateIR(); // get baseline IR reading

  // mark the border walls
  for(uint8_t x = 0; x < n; x++) {
    maze[x][0].data |= 0b1 << (WALLS + 1);
    maze[x][n - 1].data |= 0b1 << (WALLS + 1);
  }
  for(uint8_t y = 0; y < n; y++) {
    maze[0][y].data |= 0b1 << WALLS;
    maze[n - 1][y].data |= 0b1 << WALLS;
  }

  blueLEDOn(); // signal initialization complete
}

void loop() {
  /* Program superloop */
  
  if(!(robot_node->data & (0b1 << VISITED))) { // if robot node is unexplored...
    robot_node->data |= 0b1 << VISITED; // mark it as explored
    Serial.println("Found new node!");

    updateWalls(); // mark newly discovered walls
  }

  robot_node->data |= 0b1 << CURRENT; // mark robot node as current
  char msg[50];
  snprintf(msg, 50, "I'm @(%d, %d), pointing along (%d, %d)", getPos(robot_node).x, getPos(robot_node).y, robot_hdg.x, robot_hdg.y);
  Serial.println(msg);

  node *next_step = greedy(); // find next step towards target using a greedy algorithm
  //transmit maze // radio the entire maze to the base station
  //waitForGo(); // wait for user to acknowledge before proceeding to the next node

  cleanNodes(); // remove temporary node information
  Serial.println();
  
  if(!next_step) { // if robot is done...
    victoryDance(); // perform victory dance
    
    while(1); // wait forever
  }
  else {
    robot_hdg = setHDG(calcHDG(robot_node, next_step)); // otherwise turn towards the next node
    uint8_t status = advanceOne(); // try to reach the next node
    if(status == 0) { // if next node reached sucessfully...
      robot_node = next_step; // update robot node
    }
    else {
      backUp(); // otherwise (if another robot blocks the path), reverse to the starting node
      maze[getPos(robot_node).x + (robot_hdg.x == 1)][getPos(robot_node).y + (robot_hdg.y == 1)].data |= 0b1 << (TEMP_WALLS + (robot_hdg.y != 0)); // mark a temporary wall blocking the path
    }
  }
}
