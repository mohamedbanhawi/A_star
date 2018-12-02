#include <iostream>
#include <iomanip>
#include "astar.hpp"
#include <sstream>

//////////////////////////////
///         Action         ///
//////////////////////////////

action::action(int64_t x, int64_t y)
{
    dx = x;
    dy = y;
    cost = 1;
    _set_name();
}

action::action(int64_t x, int64_t y, int64_t c)
{
    dx = x;
    dy = y;
    cost = c;
    _set_name();
}

action::action() {};

action::~action() {};

void action::_set_name()
{
    if ((dx == -1) && (dy == 0))
        name = '^';
    if ((dx == 0) && (dy == -1))
        name = '<';
    if ((dx == 1) && (dy == 0))
        name = 'v';
    if ((dx == 0) && (dy == 1))
        name = '>';
}


//////////////////////////////
///         Node           ///
//////////////////////////////

node::node(const int64_t& grid_x,
            const int64_t& grid_y)
{ 
    x = grid_x;
    y = grid_y;
    g = 0;
    h = 0;
    f = 0;
    order = -1;
    closed = false;
    state = State::FREE;
    action_here = action();
}

node::node(const int64_t& grid_x, 
            const int64_t& grid_y, 
            const int64_t& grid_val)
{
    x = grid_x;
    y = grid_y;
    g = 0;
    h = 0;
    f = 0;
    order = -1;
    closed = false;
    action_here = action();
    switch (grid_val)
    {
        case 0:
            state = State::FREE;
            break;
        case -1:
            state = State::OCCUPIED;
            break;
        case 1:
            state = State::START;
            order = 1;
            break;
        case 2:
            state = State::GOAL;
            break;
        default:
            state = State::FREE;
            break;
    }
}   

node::node(){}
node::~node(){}

bool node::Expand() const
{   // don't expand a closed node or an occupied node
    return !(state == State::OCCUPIED || closed);
}

bool node::operator<(const node& n) const
{
    if ((f > n.f))
    {
        return true; // higher cost is lower priority
    }

    return false;
}

bool node::operator==(const node& n) const
{
    if ((x == n.x) && (y == n.x))
    {
        return true;
    }

    return false;
}

int64_t node::CostTo(const node& n) const
{
    return (abs(x-n.x) + abs(y-n.y));
}

void node::predict(node& NextNode, 
                    const action& delta) const
{   
    NextNode.x = x + delta.dx;
    NextNode.y = y + delta.dy;
}

std::string node::AsString()
{
    std::stringstream ss;
    ss << "("<< x << ","<< y<<")" << 
        ", State: "<< state <<
        ", Closed "<< int(closed) <<
        ", f: " << f << 
        "= g: " << g << 
        "+ h: " << h;
    return ss.str();
}


//////////////////////////////
///    A-Star Algorithm    ///
//////////////////////////////

a_star::a_star()
{
    _initalised = false;
    _goal_reached = false;
    _resigned = false;
    _actions = {action(-1,0),
                action(0,-1),
                action(1,0),
                action(0,1)};


}
a_star::~a_star() {}

bool a_star::Init(map& input_map) 
{   
    std::cout << "a_star::Init" << std::endl;
    _map = input_map;
    int row;
    int col;
    // create grid
    for (row = 0; row < input_map.size(); row++)
    {   
        std::vector<node> grid_row;
        for (col = 0; col < input_map[0].size(); col++)
        {    
            grid_row.push_back(node(row, col, input_map[row][col]));
            std::cout << std::setw(4) << input_map[row][col];
            if (input_map[row][col] == 1)
            {
                _Start_node = grid_row.back();

            }
            else if (input_map[row][col] == 2)
            {
                _Goal_node = grid_row.back();
                
            }
        }
        std::cout << std::endl;
        _grid.push_back(grid_row);
    }
    std::cout << std::endl;
    std::cout << "_Start_node" << _Start_node.AsString() << std::endl;
    std::cout << "_Goal_node " << _Goal_node.AsString() << std::endl;
    _grid_y_max = col; // rows
    _grid_x_max = row; // columns

    // estimate heuristic function
    // create grid
    for (int row = 0; row < _grid_x_max; row++)
    {   

        for (int col = 0; col < _grid_y_max; col++)
        {   
            _grid[row][col].h = _grid[row][col].CostTo(_Goal_node);
            _grid[row][col].f = _grid[row][col].g + _grid[row][col].h;
        }
    }

    _initalised = true;
    return 1; 
}

bool a_star::InGrid(node& Node)
{   
    if ((Node.x >= _grid_x_max) 
        ||
        (Node.x < 0)
        ||
        (Node.y >= _grid_y_max) 
        ||
        (Node.y < 0))
    {   
        return false; // invalid node position
    }
    return true;
}


bool a_star::Search() 
{ 
 if (!_initalised)
 {
    return _initalised; // map not loaded
 }

 int count = 0;
 node Current = _grid[_Start_node.x][_Start_node.y];
 Current.closed = true;
 _grid[Current.x][Current.y] = Current;
 _open.push(Current); 
 
 while ((!_resigned) && (!_goal_reached)) // keep searching
 {
    if (_open.empty())
    {
        _resigned = true; // no more nodes to explore
        return 0; // fail
    }
    else
    {
        Current = _open.top(); // lowest cost = highest priority
        std::cout << "current" << Current.AsString()<< std::endl;
        ShowCost(Current);
        ShowGrid(Current);
        _open.pop(); // remove from queue

        Current.order = count;
        Current.closed = true;
        _grid[Current.x][Current.y] = Current;
        count++;
        if (Current == _Goal_node)
        {
            _goal_reached = true;
            return 1; // success
        }
        else 
        {        
           // attempt possible actions
           for (int a = 0; a < _actions.size();a++) 
           {    
                node NextNode = node();
                Current.predict(NextNode, _actions[a]);
             if (InGrid(NextNode)) // try to retrieve node from map
                {   
                    NextNode = _grid[NextNode.x][NextNode.y];
                    std::cout << "Next" << NextNode.AsString()<< std::endl;
                    if (NextNode.Expand()) // expandable node
                    {   
                        _grid[NextNode.x][NextNode.y].g = Current.g + 1;
                        _grid[NextNode.x][NextNode.y].f = _grid[NextNode.x][NextNode.y].h  + _grid[NextNode.x][NextNode.y].g;
                        _grid[NextNode.x][NextNode.y].action_here = _actions[a];
                        _grid[NextNode.x][NextNode.y].closed = true;
                        _open.push(_grid[NextNode.x][NextNode.y]);
                        std::cout << "Added to open queue" << std::endl;
                    }
                }

           } 
        }
    }
 }


if (_goal_reached) 
{
    return 1;
} // success

return 0; // fail
}

void a_star::ShowGrid() const
{    

    // width
    const int w = 4;

    for (int row = 0; row < _grid_x_max; row++)
    {   

        for (int col = 0; col < _grid_y_max; col++)
        {   
            node Node = _grid[row][col];
            std::cout << std::setw(w) << Node.order;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void a_star::ShowGrid(const node& current) const
{    

    // width
    const int w = 4;

    for (int row = 0; row < _grid_x_max; row++)
    {   

        for (int col = 0; col < _grid_y_max; col++)
        {   
            if (_grid[row][col] == current)
            {
                std::cout << "  **";
            }
            else if (_grid[row][col] == _Goal_node)
            {
                std::cout << "  xx";
            }
            else if (_grid[row][col] == _Start_node)
            {
                std::cout << "  oo";
            }
            else
            {
                node Node = _grid[row][col];
                std::cout << std::setw(w) << Node.order;
            }

        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void a_star::ShowCost(const node& current) const
{    

    // width
    const int w = 4;

    for (int row = 0; row < _grid_x_max; row++)
    {   

        for (int col = 0; col < _grid_y_max; col++)
        {   
            if (_grid[row][col] == current)
            {
                std::cout << "  **";
            }
            else if (_grid[row][col] == _Goal_node)
            {
                std::cout << "  xx";
            }
            else if (_grid[row][col] == _Start_node)
            {
                std::cout << "  oo";
            }
            else
            {
                node Node = _grid[row][col];
                std::cout << std::setw(w) << Node.f;
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void a_star::ShowCost() const
{    

    // width
    const int w = 4;

    for (int row = 0; row < _grid_x_max; row++)
    {   

        for (int col = 0; col < _grid_y_max; col++)
        {   
            node Node = _grid[row][col];
            std::cout << std::setw(w) << Node.f;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

bool a_star::UpdatePath() 
{
 return true;
}



