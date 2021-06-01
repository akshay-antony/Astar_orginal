//
// Created by ARYA-PC on 30-05-2021.
//

#ifndef UNTITLED_ASTAR_H
#define UNTITLED_ASTAR_H
#include<vector>
#include<utility>
//#include<unordered_set>
#include <bits/stdc++.h>
#include<queue>
#include<tuple>
#include<unordered_map>
#include<cmath>
#include<queue>
#include<iostream>
#include<limits>
#include<cfloat>
using namespace std;
struct Point
{
    float x;
    float y;
};
struct neighbour
{   float x;//x coordinate
    float y;//y coordinate
    float f,h,g;
};
struct Cell
{   float p1;//x coordinate of parent
    float p2;//y coordinate of parent
    float f,h,g;
};

class AStar
{
private:
    vector<vector<float>> Graph;
    Point start_point;
    Point goal_point;
    float hieght;
    float width;
    set<pair<float,float>> closed_points;
    priority_queue<tuple<float,float,float>,vector<tuple<float,float,float>>,greater<tuple<float,float,float>>> open_points;
    map<pair<float,float>,Cell>  all_points;
public:
    vector<pair<float,float>> path;
    void initialize();
    bool is_not_obstacle(float x,float y);
    AStar(float h, float w,Point sp,Point gp,vector<Point>& obtacles);
    int find_path();
    float find_h(float x,float y);
    //vector<neighbour> find_neighbour(neighbour &n);
    bool is_valid(float x, float y);
    bool is_destination(float x,float y);
    bool is_in_closed_set(float x,float y);
    void trace_back(float x,float y);
    void print_graph();
    void print_all_points();
};



#endif //UNTITLED_ASTAR_H
