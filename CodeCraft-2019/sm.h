#include<iostream>
#include<cstring>
#include<cstdlib>
#include<cstdio>
#include<cmath>
#include<algorithm>
#include<vector>
#include<list>
#include<map>
using namespace std;

struct SM_CAR{
	int id;//车辆id 
	int lim;//限速 
	int pos;//在当前道路上已行驶的距离，起始为1 
	bool v;//已到达终结状态 
	int sta;//车辆状态(0=还没出发，1=在路上，2=到达目的地)
	
	vector<int> path;//路径 
	int now;//path[now]为当前的目标点 
	
	SM_CAR(int,int);
};

struct SM_ROAD{
	int id;//道路id
	int lth;//长度
	int lim;//限速
	int sid;//起点路口
	int tid;//终点路口
	int fst;//第一个要行动的车的方向 
	int lst;//未到达终结状态的车道数 
	bool v;//有车辆更新 
	SM_ROAD *oroad[3];
	SM_ROAD *iroad[3]; 
	
	vector<list<SM_CAR*>> pass;//车道
	list<SM_CAR*> wait;//等待从车库进入道路的车 
	int now;//pass[now]为当前需要处理的车道 
	
	SM_ROAD(int,int,int,int,int,int);
	void addcar();
	void updatepass(list<SM_CAR*>&);
	void update1();
	void update2();
	void update3(); 
};

struct SM_CROSS{
	int id;//路口id 
	int rid[4];//出入道路id，init结束后按id由小到大排序 
	SM_ROAD *iroad[4];//进路口的道路，init结束后按id由小到大排序 
	SM_ROAD *oroad[4];//出路口的道路，init结束后按id由小到大排序
	
	SM_CROSS(int,int[4]);
	void update2();
};

struct SM{
	int now;//当前时间 
	bool v;//结束标记，所有车都到达目的地 
	vector<SM_CAR> car;//车 
	vector<SM_CAR*> error;//出错的车，仅考虑到达路口的那些 
	vector<SM_ROAD> road;//单向道路 
	vector<SM_CROSS> cross;//路口 
	map<int,SM_CAR*> carmap;//车id映射表 
	map<int,SM_CROSS*> crossmap;//路口id映射表 
	
	void init(const char *fcar,const char *fcross,const char *froad);
	void update();
	void carrun(int,const vector<int>&);
};
