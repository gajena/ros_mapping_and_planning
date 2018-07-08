#include "ros/ros.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

using namespace std;
using namespace cv;

const int ALLOW_VERTEX_PASSTHROUGH = 1;
const int NODE_FLAG_CLOSED = -1;
const int NODE_FLAG_UNDEFINED = 0;
const int NODE_FLAG_OPEN = 1;

const int NODE_TYPE_ZERO = 0;
const int NODE_TYPE_OBSTACLE = 1;
const int NODE_TYPE_START = 2;
const int NODE_TYPE_END = 3;

const int G_DIRECT = 10;
const int G_SKEW = 14;

class MapNode
{
  public:
    int x = -1;
    int y = -1;
    int h = 0;
    int g = 0;
    int type = NODE_TYPE_ZERO;
    int flag = NODE_FLAG_UNDEFINED;
    MapNode *parent = 0;

    MapNode() {}

    MapNode(int x, int y, int type = NODE_TYPE_ZERO, int flag = NODE_FLAG_UNDEFINED, MapNode *parent = 0)
    {
        this->x = x;
        this->y = y;
        this->type = type;
        this->flag = flag;
        this->parent = parent;
    }

    int f()
    {
        return g + h;
    }
};

class MapSize
{
  public:
    unsigned long width = 0;
    unsigned long height = 0;
    unsigned long size = 0;

    MapSize() {}

    MapSize(unsigned long width, unsigned long height)
    {
        this->width = width;
        this->height = height;
        this->size = width * height;
    }
};


MapSize mapSize;
vector<MapNode> mapData;
vector<MapNode *> openList;

MapNode *startNode;
MapNode *targetNode;

MapNode *mapAt(int x, int y);

vector<MapNode *> neighbors(MapNode *node);

int computeH(MapNode *node1, MapNode *node2);

int computeG(MapNode *node1, MapNode *node2);

vector<MapNode *> find();

void drawOpenList();

/**	The standard heuristic is the Manhattan distance.
 *	Look at your cost function and see what the least cost is
 *	for moving from one space to another.
 *	The heuristic should be cost times manhattan distance: */
inline int manhattan_distance(MapNode *node1, MapNode *node2)
{
    return abs(node2->x - node1->x) + abs(node2->y - node1->y);
}

/**	If on your map1 you allow diagonal movement, then you need a different heuristic.
 *	The Manhattan distance for (4 east, 4 north) will be 8.
 *	However, you could simply move (4 northeast) instead, so the heuristic should be 4.
 *	This function handles diagonals: */
inline int diagonal_distance(MapNode *node1, MapNode *node2)
{
    return max(abs(node2->x - node1->x), abs(node2->y - node1->y));
}

nav_msgs::OccupancyGrid map_; 
void mapcb(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_=*msg;
}

geometry_msgs::PoseArray waypoints_;
geometry_msgs::Pose pose_;
double offset_x ,offset_y;
    
    
nav_msgs::Odometry odo_;
void odocb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odo_ = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, mapcb);
    ros::Subscriber odo_sub = nh.subscribe < nav_msgs::Odometry>("/rovio/odometry", 10, odocb);

    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseArray>("/waypoints", 10);

    ros::Rate loop_rate(30);

    int x_des,y_des;
    
    while (ros::ok())
    {
        mapSize = MapSize(map_.info.width, map_.info.height);
        mapData = vector<MapNode>(mapSize.size);
        // cout << "MapSize(" << mapSize.width << ", " << mapSize.height << ", " << mapSize.size << ")" << endl;
        offset_x =  map_.info.origin.position.x;
        offset_y = map_.info.origin.position.y;
        // cout << "MapSize(" << offset_x << ", " << offset_y << ", " << mapSize.size << ")" << endl;
        if (map_.info.width > 0)
        {
            for (int y = 0; y < map_.info.height; y++)
            {
                for (int x = 1; x < map_.info.width-1; x++)
                {
                    if(map_.data[(map_.info.width) * y + x] == 0)
                    {
                        x_des = x;
                        y_des = y; 
                    }

                }
            }

            for (int y = 0; y < map_.info.height; y++)
            {
                for (int x = 0; x < map_.info.width; x++)
                {
                    if ((x == (int)((odo_.pose.pose.position.x - offset_x) * 20.0f)) && (y == (int)((odo_.pose.pose.position.y-offset_y) * 20.0f)))
                    {
                    // cout << "debug=" << (x==(int)(odo_.pose.pose.position.x- offset_x) * 20) <<","<<( y==(int)(odo_.pose.pose.position.y - offset_y) * 20) << "," << x << "," << y << endl;
                        MapNode node(x, y, NODE_TYPE_START);
                        mapData[y * mapSize.width + x] = node;
                        startNode = &mapData[y * mapSize.width + x];
                    }
                    else if ((x == (int)((1.0 - offset_x) * 20.0f)) && (y == (int)((-0.6-offset_y) * 20.0f)))
                    {
                        MapNode node(x, y, NODE_TYPE_END);
                        mapData[y * mapSize.width + x] = node;
                        targetNode = &mapData[y * mapSize.width + x];
                    }
                    else if (map_.data[((map_.info.width) * y )+ x] < 50  && map_.data[((map_.info.width) * y )+ x]>-1)
                    {
                        mapData[(map_.info.width) * y + x] = MapNode(x, y, NODE_TYPE_ZERO);
                    }
                   
                    else if (map_.data[(map_.info.width) * y + x] > 51 && map_.data[((map_.info.width) * y )+ x] == -1)
                    {
                        mapData[(map_.info.width) * y + x] = MapNode(x, y, NODE_TYPE_OBSTACLE);
                        int obs_size =1 ;
                        {
                        if (map_.data[(map_.info.width) * (y-1) + x] < 50 && y>(obs_size))
                        {
                            for (int i = 1; i < obs_size && y>i ; i++)
                                mapData[(map_.info.width) * (y - i) + x] = MapNode(x, y - i, NODE_TYPE_OBSTACLE);
                        }
                        else if (map_.data[(map_.info.width) * (y + 1) + x] < 50 && (y+obs_size) < map_.data[(map_.info.height)])
                        {
                            for (int i = 1; i < obs_size; i++)
                                mapData[(map_.info.width) * (y + i) + x] = MapNode(x, y + i, NODE_TYPE_OBSTACLE);
                                // cout<<"(debug"<<x<<","<<y<<")"<<endl;
                        }
                        else if (map_.data[(map_.info.width) * (y ) + (x+1)] < 50 && (x+obs_size) < map_.data[(map_.info.width)])
                        {
                            for (int i = 1; i < obs_size; i++)
                                mapData[(map_.info.width) * (y) + (x + i)] = MapNode(x + i, y, NODE_TYPE_OBSTACLE);
                        }
                        else if (map_.data[(map_.info.width) * (y) + (x - 1)] < 50 && x>(obs_size))
                        {
                            for (int i = 1; i < obs_size  && x> i; i++)
                                mapData[(map_.info.width) * (y) + (x - i)] = MapNode(x - i, y, NODE_TYPE_OBSTACLE);
                        }
                        }
                    }
                    else
                    {
                        mapData[y * mapSize.width + x] = MapNode(x, y, NODE_TYPE_OBSTACLE);
                    }
                }
            }

            // for (int y = 0; y < map_.info.height; y++)
            // {
            //     for (int x = 0; x < map_.info.width; x++)
            //     {
            //         cout << mapAt(x, y)->type << " ";
            //     }
            //     cout << endl;
            // }

            openList.push_back(startNode);
            vector<MapNode *> path = find();
            mapData.clear();

            waypoints_.header.stamp = ros::Time::now();
            waypoints_.header.frame_id = "world";
            
            
        }
        setpoint_pub.publish(waypoints_);
        waypoints_.poses.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


void drawOpenList()
{
    for (int i = 0; i < openList.size(); i++)
    {
        MapNode *node = openList[i];
        if (node == startNode || node == targetNode)continue;
    }
}

vector<MapNode *> find()
{
    vector<MapNode *> path;
    cout << "Finding started!" << endl;
    int iteration = 0;
    MapNode *node;
    MapNode *reversedPtr = 0;
    while (openList.size() > 0)
    {
        node = openList.at(0);

        for (int i = 0, max = openList.size(); i < max; i++)
        {
            if (openList[i]->f() <= node->f() && openList[i]->h < node->h)
            {
                node = openList[i];
            }
        }
        openList.erase(remove(openList.begin(), openList.end(), node), openList.end());
        node->flag = NODE_FLAG_CLOSED;
        // cout << iteration++ << endl;
        // cout << "   Current node " << node->x << ", " << node->y << " ..." << endl;
       

        if (node->parent != 0)
        {
            // cout << "       ... parent " << node->parent->x << ", " << node->parent->y << endl;
        
        }
        if (node == targetNode)
        {
            cout << "Reached the target node." << endl;
            reversedPtr = node;
            iteration = 0;
            openList.clear(); 
            break;
        }
        vector<MapNode *> neighborNodes = neighbors(node);
        // cout << "       ... has " << neighborNodes.size() << " neighbors" << endl;
        for (int i = 0; i < neighborNodes.size(); i++)
        {
            MapNode *_node = neighborNodes[i];
            if (_node->flag == NODE_FLAG_CLOSED || _node->type == NODE_TYPE_OBSTACLE)
            {
                continue;
            }
            int g = node->g + computeG(_node, node);
            if (_node->flag == NODE_FLAG_UNDEFINED || g < _node->g)
            {
                _node->g = g;
                _node->h = computeH(_node, targetNode);
                _node->parent = node;
                if (_node->flag != NODE_FLAG_OPEN)
                {
                    _node->flag = NODE_FLAG_OPEN;
                    openList.push_back(_node);
                }
            }
        }
        drawOpenList();
        if (openList.size() <= 0)
            break;
    }
    if (reversedPtr == 0)
    {
        cout << "Target node is unreachable." << endl;
    }
    else
    {
        MapNode *_node = reversedPtr;
        while (_node->parent != 0)
        {
            path.push_back(_node);
            pose_.position.x= ((double)_node->parent->x / 20.0 + offset_x );
            pose_.position.y = ((double)_node->parent->y / 20.0 + offset_y);
            waypoints_.poses.push_back(pose_);
            
            _node = _node->parent;
        }
        reverse(path.begin(), path.end());
        reverse(waypoints_.poses.begin(), waypoints_.poses.end());
    }
    return path;
}

vector<MapNode *> neighbors(MapNode *node)
{
    vector<MapNode *> available;
    MapNode *_node;

    // L
    if ((_node = mapAt(node->x - 1, node->y)) != 0)
        available.push_back(_node);
    // T
    if ((_node = mapAt(node->x, node->y - 1)) != 0)
        available.push_back(_node);
    // R
    if ((_node = mapAt(node->x + 1, node->y)) != 0)
        available.push_back(_node);
    // B
    if ((_node = mapAt(node->x, node->y + 1)) != 0)
        available.push_back(_node);

    if (ALLOW_VERTEX_PASSTHROUGH)
    {
        // LT
        if ((_node = mapAt(node->x - 1, node->y - 1)) != 0)
            available.push_back(_node);
        // RT
        if ((_node = mapAt(node->x + 1, node->y - 1)) != 0)
            available.push_back(_node);
        // RB
        if ((_node = mapAt(node->x + 1, node->y + 1)) != 0)
            available.push_back(_node);
        // LB
        if ((_node = mapAt(node->x - 1, node->y + 1)) != 0)
            available.push_back(_node);
    }

    return available;
}

int computeH(MapNode *node1, MapNode *node2)
{
    // return abs(node1->x - node2->x) + abs(node1->y - node2->y);
    if (ALLOW_VERTEX_PASSTHROUGH)
    {
        return diagonal_distance(node1, node2) * G_SKEW;
    }
    else
    {
        return manhattan_distance(node1, node2) * G_DIRECT;
    }
}

int computeG(MapNode *node1, MapNode *node2)
{
    int dX = abs(node1->x - node2->x);
    int dY = abs(node1->y - node2->y);
    if (dX > dY)
    {
        return 14 * dY + 10 * (dX - dY);
    }
    else
    {
        return 14 * dX + 10 * (dY - dX);
    }
}

MapNode *mapAt(int x, int y)
{
    if (x < 0 || y < 0 || x >= mapSize.width || y >= mapSize.height)
        return 0;
    return &mapData[y * mapSize.width + x];
}