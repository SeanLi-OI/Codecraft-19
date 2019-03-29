//Author: Lixiang
#include <stdio.h>
#include <queue>
#include <stack>
#include <map>
#include <utility>
#include <string>
#include <memory.h>
#include <iostream>
#include "util.h"
#include "sm.h"

class MAIN
{
  private:
    std::vector<CAR> cars;
    std::vector<CROSS> crosss;
    std::vector<ROAD> roads;
    std::map<int, int> cross_map;
    std::map<int, bool> is_cal;
    MYMAP mymap;
    MYPATH mypath;
    int cross_num;
    FILE *fout;
    SM sm;

  public:
    void input(const char *car_inputfile, const char *cross_inputfile, const char *road_inputfile)
    {
        sm.init(car_inputfile,cross_inputfile,road_inputfile);
        char buf[50];
        int id, from, to, speed, planTime, roadId[4], length, channel, isDuplex;
        FILE *fin = fopen(car_inputfile, "r");
        
        fgets(buf, 50, fin);
        while (fgets(buf, 50, fin) != NULL)
            cars.push_back(CAR(buf));
        fclose(fin);

        fin = fopen(cross_inputfile, "r");
        fgets(buf, 50, fin);
        while (fgets(buf, 50, fin) != NULL)
            crosss.push_back(CROSS(buf));
        fclose(fin);

        fin = fopen(road_inputfile, "r");
        fgets(buf, 50, fin);
        while (fgets(buf, 50, fin) != NULL)
            roads.push_back(ROAD(buf));
        fclose(fin);
    }
    void preprocess()
    {
        cross_num = 0;
        for (auto it : crosss)
            cross_map[it.id] = cross_num++;
        mymap.init(cross_num);
        mypath.init(cross_num);
        for (auto it : roads)
        {
            mymap.addEdge(it.id, cross_map[it.from], cross_map[it.to], it.length, it.speed, it.channel);
            if (it.isDuplex)
                mymap.addEdge(it.id, cross_map[it.to], cross_map[it.from], it.length, it.speed, it.channel);
        }
    }
    void Dijkstra(int from, int to)
    {
        std::priority_queue<HEAP_NODE> heap;
        std::pair<int, int> *ans_path = new std::pair<int, int>[cross_num];
        int *distance = new int[cross_num];
        bool *visited = new bool[cross_num];
        int now_id;
        memset(distance, 0x3f, sizeof(int) * cross_num);
        memset(visited, 0, sizeof(bool) * cross_num);
        distance[from] = 0;
        heap.push(HEAP_NODE(from, distance[from]));
        while (!heap.empty())
        {
            now_id = heap.top().id;
            heap.pop();
            if (visited[now_id])
                continue;
            visited[now_id] = 1;
            for (auto it : mymap.ADJL[now_id])
            {
                if (!visited[it.to] && distance[it.to] > distance[now_id] + it.length)
                {
                    distance[it.to] = distance[now_id] + it.length;
                    heap.push(HEAP_NODE(it.to, distance[it.to]));
                    ans_path[it.to] = std::make_pair(now_id, it.id);
                }
            }
        }
        is_cal[from] = 1;
        mypath.path[from] = ans_path;
        delete distance;
        delete visited;
    }
    void print_path(int from, int to)
    {
        std::stack<int> ans_stack;
        std::pair<int, int> *ans_path = mypath.path[from];
        int now_id = to;
        while (now_id != from)
        {
            ans_stack.push(ans_path[now_id].second);
            now_id = ans_path[now_id].first;
        }
        while (!ans_stack.empty())
        {
            fprintf(fout, ",%d", ans_stack.top());
            ans_stack.pop();
        }
    }
    std::vector <int> solve_path(int from,int to){
        std::stack<int> ans_stack;
        std::pair<int, int> *ans_path = mypath.path[from];
        std::vector<int> ans;
        int now_id = to;
        while (now_id != from)
        {
            ans_stack.push(ans_path[now_id].second);
            now_id = ans_path[now_id].first;
        }
        while (!ans_stack.empty())
        {
            ans.push_back(ans_stack.top());
            /*fprintf(fout, ",%d", ans_stack.top());*/
            ans_stack.pop();
        }
        return ans;
    }
    void solve(const char *output_file)
    {
        fout = fopen(output_file, "w");
        int now=0;
        for (auto it : cars)
        {
            fprintf(fout, "(%d,%d", it.id, it.planTime+now);
            now+=5;
            if (!is_cal[cross_map[it.from]])
                Dijkstra(cross_map[it.from], cross_map[it.to]);
            print_path(cross_map[it.from], cross_map[it.to]);
            fprintf(fout, ")\n");
            //printf("(%d,%d", it.id, it.planTime);
            //solve_path(cross_map[it.from], cross_map[it.to]);
            //printf(")\n");
            //sm.carrun(it.id,solve_path(cross_map[it.from], cross_map[it.to]));
            
            //printf("%d\n",it.id);
        }
        /*sm.update();
        for(auto it:sm.error)
            fprintf(fout,"%d\n",it->id);*/
        fclose(fout);
    }
};
int main(int argc, char *argv[])
{
    std::cout << "Begin" << std::endl;

    if (argc < 5)
    {
        std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
        exit(1);
    }

    std::string carPath(argv[1]);
    std::string roadPath(argv[2]);
    std::string crossPath(argv[3]);
    std::string answerPath(argv[4]);

    std::cout << "carPath is " << carPath << std::endl;
    std::cout << "roadPath is " << roadPath << std::endl;
    std::cout << "crossPath is " << crossPath << std::endl;
    std::cout << "answerPath is " << answerPath << std::endl;

    MAIN Main;
    Main.input(carPath.c_str(), crossPath.c_str(), roadPath.c_str());
    Main.preprocess();
    Main.solve(answerPath.c_str());
    return 0;
}
/*int main()
{
    MAIN Main;
    Main.input("car.txt", "cross.txt", "road.txt");
    Main.preprocess();
    Main.solve("answer.txt");
    return 0;
}*/