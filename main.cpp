#include <iostream>
#include "AStar.h"
int main()
{
    float hieght=8;
    float width=8;
    Point st_p={1,2};
    Point en_p={6,7};
    vector<Point> input_obstacles;
    input_obstacles.push_back({2,2});
    input_obstacles.push_back({2,3});
    AStar astar_object(hieght,width,st_p,en_p,input_obstacles);
    if(astar_object.is_valid(st_p.x,st_p.y) && astar_object.is_valid(en_p.x,en_p.y))
    {
        astar_object.initialize();
        astar_object.find_path();
    }
    else
    {
        cout<<"\n Obstacle at start or goal Point...";
    }
    //astar_object.print_graph();
    astar_object.print_all_points();
    for(int i=0;i<astar_object.path.size();++i)
    {
        cout<<"\n"<<astar_object.path[i].first<<"   "<<astar_object.path[i].second<<"\n";
    }
}