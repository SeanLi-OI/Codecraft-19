#include <stdio.h>
#include "util.h"
#include <utility>
#include <vector>
//CAR implement
CAR::CAR(char buf[])
{
    sscanf(buf, "(%d,%d,%d,%d,%d)", &id, &from, &to, &speed, &planTime);
}

//CROSS implement
CROSS::CROSS(char buf[])
{
    sscanf(buf, "(%d,%d,%d,%d,%d)", &id, &roadId[0], &roadId[1], &roadId[2], &roadId[3]);
}

//ROAD implement
ROAD::ROAD(char buf[])
{
    sscanf(buf, "(%d,%d,%d,%d,%d,%d,%d)\n", &id, &length, &speed, &channel, &from, &to, &isDuplex);
}

//ADJL_NODE implement
ADJL_NODE::ADJL_NODE(int _id, int _to, int _length, int _speed, int _channel)
{
    id = _id;
    to = _to;
    length = _length;
    speed = _speed;
    channel = _channel;
}
//MYMAP implement
void MYMAP::init(int NODE_NUM)
{
    ADJL = new std::vector<ADJL_NODE>[NODE_NUM];
}
void MYMAP::addEdge(int id, int from, int to, int length, int speed, int channel)
{
    ADJL[from].push_back(ADJL_NODE(id, to, length, speed, channel));
}

//MYPATH implement
void MYPATH::init(int NODE_NUM)
{
    path = new std::pair<int, int> *[NODE_NUM];
}

//HEAP_NODE implement
HEAP_NODE::HEAP_NODE(int _id, int _distance)
{
    id = _id;
    distance = _distance;
}
bool HEAP_NODE::operator<(HEAP_NODE a) const
{
    return a.distance > distance;
}