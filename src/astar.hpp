
#ifndef ASTAR
#define ASTAR

#include <functional>
#include <queue>

//////////////////////////////
///         Action         ///
//////////////////////////////

class action
{
  private:
    void _set_name();

  public:
    // cartesian coordinates
    int64_t dx;
    int64_t dy;
    std::string name;
    int64_t cost;
    // constructor
    action(int64_t, int64_t);
    action(int64_t, int64_t, int64_t);
    action();
    ~action();
};

//////////////////////////////
///         Node           ///
//////////////////////////////

class node 
{
  public:
    //// properties
    // cartesian coordinates
    int64_t x;
    int64_t y;
    // cost
    int64_t g; // expansion cost
    int64_t h; // heuristic cost
    int64_t f; // total cost = g + h
    int64_t order; // order
    // search properties
    bool closed; // cost evaluated
    action action_here; // action to reach here
    std::string policy; //

    enum State
   {
      START,
      GOAL,
      OCCUPIED,
      FREE
   };

    State state; // obstacle node


    /*
    * Constructor
    */
    node(const int64_t&, const int64_t&);
    node(const int64_t&, const int64_t&, const int64_t&);
    node();
    /*
    * Destructor.
    */
    virtual ~node();

    // expand from parent node
    void ExpandFrom(const node&, const action&);

    // return feasbility of expansions
    bool Expand() const;

    // operators for comparison
    bool operator<(const node&) const; // priority not cost
    bool operator==(const node&) const; 

    int64_t CostTo(const node& n) const;


    void predict(node& ,const action&) const;
    std::string AsString();

};

struct compare_nodes
{
    bool operator()(const node lhs, const node rhs) const
    {
        return !(lhs.f < rhs.f);
    }
};

//////////////////////////////
///    A-Star Algorithm    ///
//////////////////////////////

class a_star {

  private:
    //// properties
    bool _initalised;
    bool _goal_reached;
    bool _resigned;
    int _resolution;

    //// open queue
    std::priority_queue<node, std::vector<node>, compare_nodes> _open;

    //// grid
    std::vector<std::vector<node>> _grid;
    int64_t _grid_x_max;
    int64_t _grid_y_max;
    bool InGrid(node&);

    //// search conditions
    node _Start_node = node();
    node _Goal_node  = node();

    //// applicable actions
    std::vector<action> _actions;

  public:
    typedef std::vector<std::vector<int64_t>> map;
    map _map;
    /*
    * Constructor
    */
    a_star();
    a_star(int&);

    /*
    * Destructor.
    */
    virtual ~a_star();

    /*
    * Initialize a_star.
    */
    bool Init(map&);

    /*
    * Update the a_start error variables given cross track error.
    */
    bool Search();

    /*
    * Calculate the total a_start error.
    */
    bool UpdatePath();

    /*
    * print expansion
    */
    void ShowGrid() const;
    void ShowGrid(const node&) const;
    void ShowCost() const;
    void ShowCost(const node&) const;
};

#endif /* ASTAR */
