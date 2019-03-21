#ifndef UTIL_H
#define UTIL_H
#include<utility>
#include<vector>

class CAR
{
  public:
    int id;
    int from, to;
    int speed;
    int planTime;
    CAR() {}
    CAR(char buf[]);
};
class CROSS
{
  private:
    int roadId[4];

  public:
    int id;
    CROSS() {}
    CROSS(char buf[]);
};
class ROAD
{
  public:
    int id;
    int length;
    int speed;
    int channel;
    int from, to;
    int isDuplex;
    ROAD() {}
    ROAD(char buf[]);
};
class ADJL_NODE
{
  public:
    int id;
    int to;
    int length;
    int speed;
    int channel;
    ADJL_NODE() {}
    ADJL_NODE(int _id, int _to, int _length, int _speed, int _channel);
};
class MYMAP
{
  public:
    std::vector<ADJL_NODE> *ADJL;
    void init(int NODE_NUM);
    void addEdge(int id, int from, int to, int length, int speed, int channel);
};
class MYPATH
{
  private:
  public:
    std::pair<int, int> **path;
    void init(int NODE_NUM);
};
struct HEAP_NODE
{
    int id;
    int distance;
    HEAP_NODE() {}
    HEAP_NODE(int _id, int _distance);
    bool operator<(HEAP_NODE a)const;
};
#endif