#include "usc_quadrotor.h"

#define priq_purge(q) (q)->n = 1
#define priq_size(q) ((q)->n - 1)
#define NUll 0L


int map[MAP_SIZE][MAP_SIZE][MAP_SIZE];

typedef struct {
  void *data;
  int pri;
} q_elem_t;

typedef struct {
  q_elem_t *buf;
  int n, alloc;
} pri_queue_t;

pri_queue_t *priq_new(int size)
{
  if (size < 4) size = 4;
  pri_queue_t *q = (pri_queue_t*)malloc(sizeof(pri_queue_t));
  q->buf = (q_elem_t*)malloc(sizeof(q_elem_t) * size);
  q->alloc = size;
  q->n = 1;

  return q;
}

void priq_push(pri_queue_t *q, void *data, int pri)
{
  q_elem_t *b;
  int n, m;

  if (q->n >= q->alloc) {
    q->alloc *= 2;
    b = q->buf = (q_elem_t*)realloc(q->buf, sizeof(q_elem_t) * q->alloc);
  } else { b = q->buf;
  }

  n = q->n++;
  /*
   * Append at end, then increase the heap
   */
  while ((m = n / 2) && pri < b[m].pri) {
    b[n] = b[m];
    n = m;
  }
  b[n].data = data;
  b[n].pri = pri;
}

void *priq_pop(pri_queue_t *q, int *pri) {
  void *out;
  if (q->n == 1) return 0;

  q_elem_t *b = q->buf;

  out = b[1].data;
  if (pri) *pri = b[1].pri;

  /*
   * Pull last item to top, then decrease heap
   */
  --q->n;

  int n = 1, m;
  while ((m = n * 2) < q->n) {
    if (m + 1 < q->n && b[m].pri > b[m + 1].pri)
      m++;
    if (b[q->n].pri <= b[m].pri)
      break;
    b[n] = b[m];
    n = m;
  }

  b[n] = b[q->n];
  if (q->n < q->alloc / 2 && q->n >= 16)
    q->buf = (q_elem_t*)realloc(q->buf, (q->alloc /= 2) * sizeof(b[0]));
  return out;
}

void* priq_top(pri_queue_t *q, int *pri)
{
  if (q->n == 1) return 0;
  if (pri) *pri = q->buf[1].pri;
  return q->buf[1].data;
}

void priq_combine(pri_queue_t *q, pri_queue_t *q2)
{
  int i;
  q_elem_t *e = q2->buf + 1;

  for (i = q2->n - 1; i >= 1; i--, e++)
    priq_push(q, e->data, e->pri);
  priq_purge(q2);
}
/*
int check_collision()
{
}*/

/*
 * The map is just a 3D array. 1 means occupied and 0 means free
 */
typedef struct point {
  int x, y, z;
  int is_visited;
} point_t;

double cost(point_t const *p1, point_t const *p2) {
  double diff1, diff2, diff3;
  diff1 = (double)(p1->x - p2->x);
  diff2 = (double)(p1->y - p2->y);
  diff3 = (double)(p1->z - p2->z);
  double distance = sqrt(pow(diff1, 2.0) 
      + pow(diff2, 2.0) + pow(diff3, 2.0)); 
  return distance;
}

/* 
 * For now, heuristic and cost are really the same 
 */
double heuristic(point_t const *p1, point_t const *p2) {
  double diff1, diff2, diff3;
  diff1 = (double)(p1->x - p2->x);
  diff2 = (double)(p1->y - p2->y);
  diff3 = (double)(p1->z - p2->z);
  double distance = sqrt(pow(diff1, 2.0) 
      + pow(diff2, 2.0) + pow(diff3, 2.0)); 
  return distance;
}

int valid_point(point_t p) 
{
  if (p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < MAP_SIZE && p.y < MAP_SIZE && p.z < MAP_SIZE)
    return 1;
  else
    return 0; 
}
int inflate(point_t const *p, point_t *const neighbours, int *const size)
{
  int i, j, k;
  int x, y, z;
  int index;
  point_t newp;

  index = 0;
  for (i = 0; i < 3; i++) {
    switch (i) {
      case 0: x = -1;
        break;
      case 1: x = 0;
        break;
      case 2: x = 1;
        break;
    }
    for (j = 0; j < 3; j++) {
      switch(j) {
        case 0: y = -1;
          break;
        case 1: y = 0;
          break;
        case 2: y = 1;
          break;
      }
      for (k = 0; k < 3; k++) {
        switch(k) {
          case 0: z = -1;
            break;
          case 1: z = 0;
            break;
          case 2: z = 1;
            break;
        }
        newp.x = p->x + x;
        newp.y = p->y + y;
        newp.z = p->z + z;
        newp.is_visited = 0;
        if (!(newp.x == p->x && newp.y == p->y && newp.z == p->z) && valid_point(newp) && (map[newp.x][newp.y][newp.z] == 0)) {
          /* index = i*(3^2) + j*3 + k;*/
          neighbours[index].x = newp.x;
          neighbours[index].y = newp.y;
          neighbours[index].z = newp.z;
          neighbours[index].is_visited = newp.is_visited;
          /*ROS_INFO("inflated (%d, %d, %d) visited %d\n" , 
              neighbours[index].x, neighbours[index].y, neighbours[index].z, neighbours[index].is_visited);*/
          index++;
        }
      }
    }
  }
  if (size)
    *size = index;
  return 0;
  /*if (index)
    return 0;
  else
    return 1;
  */
}

int build_frontier(pri_queue_t *const pq, point_t *const ball, int const size, point_t p, point_t goal) {
  point_t *new_point = NULL;
  double distance;
  int i = 0;

  for (i = 0; i < size; i++) {
    if (!(ball[i].is_visited && new_point->x >= 0 && new_point->y >= 0 && new_point->z >= 0)) {
      new_point = (point_t*)malloc(sizeof(point_t));
      new_point->x = ball[i].x;
      new_point->y = ball[i].y;
      new_point->z = ball[i].z;
      new_point->is_visited = ball[i].is_visited;
      distance = cost(&p, new_point) + heuristic(new_point, &goal); /* The cost is just the  */
      priq_push(pq, new_point, distance);
    }

  }
  return 0;

}

int clear_frontier(pri_queue_t *pq) {
  point_t *p;
  /*int err = 0;*/
  while ((p = (point_t*)priq_pop(pq, NULL)) != NULL) {
    free(p);
  }
  return 0;

}

int goal_test(point_t p, point_t goal) {
  if (p.x == goal.x && p.y == goal.y && p.z == goal.z) {
    return 1;
  } else {
    return 0;
  }

}

bool get_path(usc_quadrotor::trajectory::Request  &req, usc_quadrotor::trajectory::Response &res){

  
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int k = 0; k < MAP_SIZE; k++) {
      for (int j = 0; j < MAP_SIZE; j++) {
        map[i][j][k] = 0;
      }
    }
  }

  res.exists = true;

  point_t start, goal, *current;
  point_t ball[26];
  
  int size; /* Size of the ball */
  /*point_t *plan;*/
  pri_queue_t *frontier = priq_new(4);
  double distance;
  int err;

  start.x = req.source[0];
  start.y = req.source[1];
  start.z = req.source[2];
   
  goal.x = req.destination[0];
  goal.y = req.destination[1];
  goal.z = req.destination[2];
  
  ROS_INFO("Astar planning starts at (%d, %d, %d) for (%d, %d, %d)", 
            start.x, start.y, start.z, goal.x, goal.y, goal.z);

  distance = cost(&start, &start) + heuristic(&start, &goal);
  priq_push(frontier, &start, distance);
  
  int old_x = start.x;
  int old_y = start.y;
  int old_z = start.z;

  while ((current = (point_t*)priq_pop(frontier, NULL)) != NULL) {
  
    // ROS_INFO("(%d, %d, %d)\n", current->x, current->y, current->z);
  
    /*  
     * If a map is saved then the is_visited attribute is redundent,
     * however it's no harm to put it here as in c++ this will definitely
     * be changed.
     */ 
    res.path.push_back( 
      100*( (current->x - old_x) < 0 ? 2 : (current->x - old_x )) + 
      10 *( (current->y - old_y) < 0 ? 2 : (current->y - old_y )) + 
          ( (current->z - old_z) < 0 ? 2 : (current->z - old_z ))
      );

    old_x = current->x;
    old_y = current->y;
    old_z = current->z;

    current->is_visited = 1;
    map[current->x][current->y][current->z] = 1;
    /* A dummy goal test that just check if all coordinates agree */
    if (!goal_test(*current, goal)) {
      /* 
       * Expand the frontier 
       */
      if ((err = inflate(current, ball, &size)) > 0)
        break;
      if (size > 0)
        build_frontier(frontier, ball, size, *current, goal);
    } else {
      /* 
       * Return the plan  
       */
      ROS_INFO("Planning has finished with astar at (%d, %d, %d)", goal.x, goal.y, goal.z);
      
      return true;
    }
  }
  res.exists = false;
  ROS_ERROR("No possible path leads from (%d, %d, %d) to (%d, %d, %d)", 
      start.x, start.y, start.z, goal.x, goal.y, goal.z);
  /* Free all the memory */
  clear_frontier(frontier);
  
  return false;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "planner");
  
  ROS_INFO("Planner is Initializing");

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("get_plan", get_path);
  
  /*
   * Initialize the map
   */
  ROS_INFO("Planner is ready");
  
  ros::spin();

  return 0;
}