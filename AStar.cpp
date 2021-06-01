//
// Created by ARYA-PC on 30-05-2021.
//

#include "AStar.h"

using namespace std;
AStar ::AStar(float h,float w,Point sp,Point gp,vector<Point>& obtacles):hieght(h),width(w), start_point(sp),goal_point(gp)
{
    //hieght=h;
    //width=w;
    //start_point=sp;
    //goal_point=gp;
    vector<float> temp1;
    for(int i=0;i<hieght;++i){
        for(int j=0;j<width;++j)
        {
            temp1.push_back(0);
        }
        Graph.push_back(temp1);
    }
    for (int i = 0; i < obtacles.size(); i++)
    {
        Point temp=obtacles[i];
        Graph[temp.x][temp.y]=1;
    }
    for(int i=0;i<hieght;++i)
    {
        for(int j=0;j<width;++j)
        {
            std::pair<float,float> temp=std::make_pair(i,j);
            Cell temp_cell;
            temp_cell={-1,-1,FLT_MAX,FLT_MAX,FLT_MAX};
            all_points[temp]=temp_cell;
        }
    }

}
void AStar ::print_graph() {
    cout<<"\n";
    for(int i=0;i<hieght;++i)
    {
        for(int j=0;j<width;++j)
        {
            cout<<Graph[i][j]<<"  ";
        }
        cout<<"\n";
    }
}
void AStar :: initialize()
{   std::pair<float,float> temp=std::make_pair(start_point.x,start_point.y);
    all_points[temp].p1=start_point.x;
    all_points[temp].p2=start_point.y;
    all_points[temp].g=0;
    all_points[temp].h=0;//check
    all_points[temp].f=0;
    std::tuple<float, float,float> temp_tuple=std::make_tuple(all_points[temp].f,start_point.x,start_point.y);
    open_points.push(temp_tuple);

}
bool AStar::  is_valid(float x,float y)
{
    if(x<hieght && y<width && x>=0 && y>=0)
        return true;
    else
        return false;
}
bool AStar::  is_not_obstacle(float x,float y)
{
    if(Graph[x][y]==0)
        return true;
    else
        return false;
}
bool AStar::  is_in_closed_set(float x,float y)
{
    std::pair<float,float> temp=std::make_pair(x,y);
    const bool is_in = closed_points.find(temp) != closed_points.end();
    return is_in;
}
bool AStar::  is_destination(float x,float y)
{
    if(x==goal_point.x && y==goal_point.y)
        return true;
    else
        return false;
}
float AStar::  find_h(float x,float y)
{
    return sqrt(pow(goal_point.x-x,2)+pow(goal_point.y-y,2));
}
void AStar::print_all_points() {
    for(int i=0;i<hieght;++i)
    { cout<<"\n";
        for(int j=0;j<width;++j)
        {
            pair<float,float> p=make_pair(i,j);
            cout<<all_points[p].f<<" ";
        }
    }
}
int AStar :: find_path()
{
    while(!open_points.empty())
    {   const tuple<float, float,float> & current_point=open_points.top();
        open_points.pop();//check
        //closed_points. add where?
        float curr_x=std::get<1>(current_point);
        float curr_y=std::get<2>(current_point);
        pair<float,float> curr_temp=make_pair(curr_x,curr_y);
       // float curr_f=std::get<0>(current_point);
        if(is_in_closed_set(curr_x,curr_y)) {
            continue;
        }
        closed_points.insert(std::make_pair(curr_x,curr_y));
        //north
        if(is_valid(curr_x,curr_y+1) && is_not_obstacle(curr_x,curr_y+1))
        {
            std::pair<float,float> temp=std::make_pair(curr_x,curr_y+1);
            if(is_destination(curr_x,curr_y+1))
            {

                all_points[temp].p1=curr_x;
                all_points[temp].p2=curr_y;
                cout<<"\n Path Found......";
                trace_back(curr_x,curr_y+1);
                return 1;
            }
            if(!is_in_closed_set(curr_x,curr_y+1))
            {
                //closed_points.insert(std::make_pair(curr_x,curr_y+1));
                float curr_h=find_h(curr_x,curr_y+1);
                float curr_g=all_points[curr_temp].g+1;
                float curr_f=curr_g+curr_h;
                if(curr_f<all_points[temp].f || all_points[temp].f==FLT_MAX)
                {
                    all_points[temp]={curr_x,curr_y,curr_f,curr_h,curr_g};
                    open_points.push(std::make_tuple(curr_f,curr_x,curr_y+1));
                }
            }
        }
        //south
        if(is_valid(curr_x,curr_y-1) && is_not_obstacle(curr_x,curr_y-1))
        {
            std::pair<float,float> temp=std::make_pair(curr_x,curr_y-1);
            if(is_destination(curr_x,curr_y-1))
            {

                all_points[temp].p1=curr_x;
                all_points[temp].p2=curr_y;
                std::cout<<"\n Path Found......";
                trace_back(curr_x,curr_y-1);
                return 1;
            }
            if(!is_in_closed_set(curr_x,curr_y-1))
            {

                float curr_h=find_h(curr_x,curr_y-1);
                float curr_g=all_points[curr_temp].g+1;
                float curr_f=curr_g+curr_h;
                if(curr_f<all_points[temp].f || all_points[temp].f==FLT_MAX)
                {
                    all_points[temp]={curr_x,curr_y,curr_f,curr_h,curr_g};
                    open_points.push(std::make_tuple(curr_f,curr_x,curr_y-1));
                }
            }
        }
        //east
        if(is_valid(curr_x+1,curr_y) && is_not_obstacle(curr_x+1,curr_y))
        {
            std::pair<float,float> temp=std::make_pair(curr_x+1,curr_y);
            if(is_destination(curr_x+1,curr_y))
            {

                all_points[temp].p1=curr_x;
                all_points[temp].p2=curr_y;
                std::cout<<"\n Path Found......";
                trace_back(curr_x+1,curr_y);
                return 1;
            }
            if(!is_in_closed_set(curr_x+1,curr_y))
            {
                //closed_points.insert(std::make_pair(curr_x,curr_y));
                float curr_h=find_h(curr_x+1,curr_y);
                float curr_g=all_points[curr_temp].g+1;
                float curr_f=curr_g+curr_h;
                if(curr_f<all_points[temp].f || all_points[temp].f==FLT_MAX)
                {
                    all_points[temp]={curr_x,curr_y,curr_f,curr_h,curr_g};
                    open_points.push(std::make_tuple(curr_f,curr_x+1,curr_y));
                }
            }
        }
        //west
        if(is_valid(curr_x-1,curr_y) && is_not_obstacle(curr_x-1,curr_y))
        {
            std::pair<float,float> temp=std::make_pair(curr_x-1,curr_y);
            if(is_destination(curr_x-1,curr_y))
            {

                all_points[temp].p1=curr_x;
                all_points[temp].p2=curr_y;
                std::cout<<"\n Path Found......";
                trace_back(curr_x-1,curr_y);
                return 1;
            }
            if(!is_in_closed_set(curr_x-1,curr_y))
            {
                //closed_points.insert(std::make_pair(curr_x,curr_y));
                float curr_h=find_h(curr_x-1,curr_y);
                float curr_g=all_points[curr_temp].g+1;
                float curr_f=curr_g+curr_h;
                if(curr_f<all_points[temp].f || all_points[temp].f==FLT_MAX)
                {
                    all_points[temp]={curr_x,curr_y,curr_f,curr_h,curr_g};
                    open_points.push(std::make_tuple(curr_f,curr_x-1,curr_y));
                }
            }
        }
        //north-east
        if(is_valid(curr_x+1,curr_y+1) && is_not_obstacle(curr_x+1,curr_y+1))
        {
            std::pair<float,float> temp=std::make_pair(curr_x+1,curr_y+1);
            if(is_destination(curr_x+1,curr_y+1))
            {

                all_points[temp].p1=curr_x;
                all_points[temp].p2=curr_y;
                std::cout<<"\n Path Found......";
                trace_back(curr_x+1,curr_y+1);
                return 1;
            }
            if(!is_in_closed_set(curr_x+1,curr_y+1))
            {
                //closed_points.insert(std::make_pair(curr_x,curr_y+1));
                float curr_h=find_h(curr_x+1,curr_y+1);
                float curr_g=all_points[curr_temp].g+1.414;
                float curr_f=curr_g+curr_h;
                if(curr_f<all_points[temp].f || all_points[temp].f==FLT_MAX)
                {
                    all_points[temp]={curr_x,curr_y,curr_f,curr_h,curr_g};
                    open_points.push(std::make_tuple(curr_f,curr_x+1,curr_y+1));
                }
            }
        }
        //north-west
        if(is_valid(curr_x-1,curr_y+1) && is_not_obstacle(curr_x-1,curr_y+1))
        {
            std::pair<float,float> temp=std::make_pair(curr_x-1,curr_y+1);
            if(is_destination(curr_x-1,curr_y+1))
            {

                all_points[temp].p1=curr_x;
                all_points[temp].p2=curr_y;
                std::cout<<"\n Path Found......";
                trace_back(curr_x-1,curr_y+1);
                return 1;
            }
            if(!is_in_closed_set(curr_x-1,curr_y+1))
            {
                //closed_points.insert(std::make_pair(curr_x,curr_y+1));
                float curr_h=find_h(curr_x-1,curr_y+1);
                float curr_g=all_points[curr_temp].g+1.414;
                float curr_f=curr_g+curr_h;
                if(curr_f<all_points[temp].f || all_points[temp].f==FLT_MAX)
                {
                    all_points[temp]={curr_x,curr_y,curr_f,curr_h,curr_g};
                    open_points.push(std::make_tuple(curr_f,curr_x-1,curr_y+1));
                }
            }
        }
        //south-east
        if(is_valid(curr_x+1,curr_y-1) && is_not_obstacle(curr_x+1,curr_y-1))
        {
            std::pair<float,float> temp=std::make_pair(curr_x+1,curr_y-1);
            if(is_destination(curr_x+1,curr_y-1))
            {

                all_points[temp].p1=curr_x;
                all_points[temp].p2=curr_y;
                std::cout<<"\n Path Found......";
                trace_back(curr_x+1,curr_y-1);
                return 1;
            }
            if(!is_in_closed_set(curr_x+1,curr_y-1))
            {
                //closed_points.insert(std::make_pair(curr_x,curr_y-1));
                float curr_h=find_h(curr_x+1,curr_y-1);
                float curr_g=all_points[curr_temp].g+1.414;
                float curr_f=curr_g+curr_h;
                if(curr_f<all_points[temp].f || all_points[temp].f==FLT_MAX)
                {
                    all_points[temp]={curr_x,curr_y,curr_f,curr_h,curr_g};
                    open_points.push(std::make_tuple(curr_f,curr_x+1,curr_y-1));
                }
            }
        }
        //south-west
        if(is_valid(curr_x-1,curr_y-1) && is_not_obstacle(curr_x-1,curr_y-1))
        {
            std::pair<float,float> temp=std::make_pair(curr_x-1,curr_y-1);
            if(is_destination(curr_x-1,curr_y-1))
            {

                all_points[temp].p1=curr_x;
                all_points[temp].p2=curr_y;
                std::cout<<"\n Path Found......";
                trace_back(curr_x-1,curr_y-1);
                return 1;
            }
            if(!is_in_closed_set(curr_x-1,curr_y-1))
            {
                //closed_points.insert(std::make_pair(curr_x,curr_y-1));
                float curr_h=find_h(curr_x-1,curr_y-1);
                float curr_g=all_points[curr_temp].g+1.414;
                float curr_f=curr_g+curr_h;
                if(curr_f<all_points[temp].f || all_points[temp].f==FLT_MAX)
                {
                    all_points[temp]={curr_x,curr_y,curr_f,curr_h,curr_g};
                    open_points.push(std::make_tuple(curr_f,curr_x-1,curr_y-1));
                }
            }
        }

    }
    cout<<"\nPath not found....";
    return 0;
}
void AStar :: trace_back(float x,float y)
{
    pair<float,float> temp=make_pair(x,y);
    path.push_back(temp);
    while(temp.first!=start_point.x || temp.second!=start_point.y)
    {
        float p1=all_points[temp].p1;
        float p2=all_points[temp].p2;
        temp=make_pair(p1,p2);
        path.push_back(temp);
    }
    reverse(path.begin(),path.end());
}

