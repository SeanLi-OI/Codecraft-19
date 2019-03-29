#include"sm.h"
using namespace std;

SM_CAR::SM_CAR(int id,int lim){
	this->id=id;
	this->lim=lim;
	this->pos=0;
	this->v=false;
	this->now=0;
	this->sta=0;
}

SM_ROAD::SM_ROAD(int id,int lth,int lim,int pn,int sid,int tid){
	this->id=id;
	this->lth=lth;
	this->lim=lim;
	this->sid=sid;
	this->tid=tid;
	this->fst=-1;
	this->lst=pn;
	this->v=false;
	for(int i=0;i<3;++i){
		this->oroad[i]=NULL;
		this->iroad[i]=NULL;
	}
	this->pass.resize(pn);
	this->now=0;
}

SM_CROSS::SM_CROSS(int id,int rid[4]){
	this->id=id;
	for(int i=0;i<4;++i)
		this->rid[i]=rid[i];
	for(int i=0;i<4;++i){
		this->iroad[i]=NULL;
		this->oroad[i]=NULL;
	}
}

//更新单个车道 
void SM_ROAD::updatepass(list<SM_CAR*> &pass){
	if(!pass.size())
		return;
	int lstpos=lth+1;
	bool lstv=false;
	while(pass.size()&&min(lim,pass.front()->lim)>=lstpos-pass.front()->pos){
		++pass.front()->sta;
		pass.pop_front();
	}
	if(!pass.size())
		return;
	for(SM_CAR *&j:pass){
		if(j->v)
			break;
		if(min(lim,j->lim)<lstpos-j->pos||lstv){
			j->pos+=min(min(lim,j->lim),lstpos-j->pos-1);
			j->v=true;	
		}
		lstpos=j->pos;
		lstv=j->v;
	}
}

//对道路内部进行更新
void SM_ROAD::update1(){
	//给所有车辆充能
	for(int i=0;i<pass.size();++i)
		if(pass[i].size())
			for(SM_CAR *&j:pass[i])
				j->v=false;
	
	//所有车辆尝试前进 
	for(int i=0;i<pass.size();++i)
		updatepass(pass[i]);
		
	fst=-1;
	for(int i=0;i<pass.size();++i)
		if(pass[i].size()&&!pass[i].front()->v){
			now=i;
			SM_CAR *tmp=pass[i].front();
			for(int j=0;j<3;++j)
				if(oroad[j]->tid==tmp->path[tmp->now])
					fst=j;
			break;
		}

	lst=0;
	for(int i=0;i<pass.size();++i)
		if(pass[i].size()&&!pass[i].front()->v)
			++lst; 
}

//经由路口引导进行更新 
void SM_ROAD::update2(){
	this->v=false;
	
	for(int &i=now;lst;i=(i+1)%pass.size())
		if(pass[i].size()&&!pass[i].front()->v){
			SM_CAR *tmp=pass[i].front();
			bool flg=true;//未进入等待状态 
			
			for(int j=0;j<3;++j)//无论结果如何，先计算fst 
				if(oroad[j]->tid==tmp->path[tmp->now])
					fst=j;
			if((fst==0&&iroad[2]&&iroad[2]->fst==1)//因转向优先级限制，进入等待状态 
			 ||(fst==2&&((iroad[0]&&iroad[0]->fst==1)||(iroad[1]&&iroad[1]->fst==0)))){
				break;
			}
			if(oroad[fst]->lim<lth-tmp->pos+1){//因限速无法进入目标道路，进入终结状态 
				pass[i].front()->v=true;
				updatepass(pass[i]);
				--lst;
				continue;
			}
			for(int j=0;j<oroad[fst]->pass.size();++j){
				list<SM_CAR*> &tarpass=oroad[fst]->pass[j];
				if(tarpass.size()!=0&&tarpass.back()->v&&tarpass.back()->pos==1)//此车道已满，看下一个 
					continue;
				if(tarpass.size()!=0&&!tarpass.back()->v){//因目标车道为等待状态，所以也进入等待状态 
					flg=false;
					break; 
				} 
				if(tarpass.size()==0||tarpass.back()->v&&tarpass.back()->pos!=1){//目标车道可以进，进入终结状态 
					tmp->v=true;
					tmp->pos=min(min(lim,oroad[fst]->lim),tmp->lim)-(lth-tmp->pos);
					++tmp->now;
					if(tarpass.size())
						tmp->pos=min(tmp->pos,tarpass.back()->pos-1);
					tarpass.push_back(tmp);
					pass[i].pop_front();
					updatepass(pass[i]);
					if(!pass[i].size()||pass[i].front()->v)
						--lst;
					break;
				}
				if(j==oroad[fst]->pass.size()-1){//目标道路堵死了，进入终结状态 
					pass[i].front()->v=true;
					updatepass(pass[i]);
					--lst;
				}
			}
			if(flg){
				v=true;
				continue;
			}
			break;
		}
		
	if(lst==0)
		fst=-1;
}

//让要出发的车辆进入道路 
void SM_ROAD::update3(){
	for(int i=0;i<pass.size()&&wait.size();++i){
		while((pass[i].size()==0||pass[i].back()->pos!=1)&&wait.size()){
			SM_CAR *tmp=wait.front();
			tmp->pos=min(min(lim,tmp->lim),pass[i].back()->pos-1);
			++tmp->now;
			pass[i].push_back(tmp);
			wait.pop_front();
		}
	}
} 

//通过路口引导道路更新 
void SM_CROSS::update2(){
	bool flg=true;
	while(flg){
		flg=false;
		for(int i=0;i<4;++i)
			if(iroad[i]&&!iroad[i]->lst!=0){
				iroad[i]->update2();
				if(iroad[i]->v)
					flg=true;
			}
	}
}

bool cmp1(SM_CROSS x,SM_CROSS y){
	return x.id<y.id;
}

//读入路口和道路数据，建立地图
void SM::init(const char *fcar,const char *fcross,const char *froad){ 
	//预备 
	FILE *fin;
	char buf[91];
	//车辆读入 
	fin=fopen(fcar,"r");
	while(fgets(buf,90,fin)!=NULL){
		if(buf[0]=='#')
			continue;
		int id,sid,tid,lim,tim;
		sscanf(buf,"(%d, %d, %d, %d, %d)",&id,&sid,&tid,&lim,&tim);
		car.push_back(SM_CAR(id,lim));
	}
	fclose(fin);

	for(auto it:car)
		carmap[it.id]=&it;
	
	//路口读入 
	fin=fopen(fcross,"r");
	while(fgets(buf,90,fin)!=NULL){
		if(buf[0]=='#')
			continue;
		int id,rid[4];
		sscanf(buf,"(%d, %d, %d, %d, %d)",&id,&rid[0],&rid[1],&rid[2],&rid[3]);
		cross.push_back(SM_CROSS(id,rid));
	}
	fclose(fin);

	for(auto it:cross)
		crossmap[it.id]=&it;

	//道路读入，统一转换为单行道路处理 
	fin=fopen(froad,"r");
	while(fgets(buf,90,fin)!=NULL){
		if(buf[0]=='#')
			continue;
		int id,lth,lim,pn,sid,tid,bi;
		sscanf(buf,"(%d, %d, %d, %d, %d, %d, %d)",&id,&lth,&lim,&pn,&sid,&tid,&bi);
		for(int i=0;i<=bi;++i){
			road.push_back(SM_ROAD(id,lth,lim,pn,sid,tid));
			swap(sid,tid);
		}
	}
	fclose(fin);
	
	//根据道路信息补全路口信息
	for(int i=0;i<road.size();++i)
		for(int j=0;j<4;++j){
			if(crossmap[road[i].sid]->rid[j]==road[i].id)
				crossmap[road[i].sid]->oroad[j]=&road[i];
			if(crossmap[road[i].tid]->rid[j]==road[i].id)
				crossmap[road[i].tid]->iroad[j]=&road[i];
		}
	//根据路口信息补全道路信息
	for(int i=0;i<cross.size();++i)
		for(int j=0;j<4;++j)
			if(cross[i].iroad[j]!=NULL)
				for(int k=0;k<3;++k){
					cross[i].iroad[j]->oroad[k]=cross[i].oroad[(j+k+1)%4];
					cross[i].iroad[j]->iroad[k]=cross[i].iroad[(j+k+1)%4];
				}
	
	//对路口的道路按id排序
	for(int i=0;i<cross.size();++i)
		for(int j=0;j<4;++j){
			int tmp=j;
			for(int k=j+1;k<4;++k)
				if(cross[i].rid[k]<cross[i].rid[tmp])
					tmp=k;
			swap(cross[i].rid[j],cross[i].rid[tmp]);
			swap(cross[i].iroad[j],cross[i].iroad[tmp]);
			swap(cross[i].oroad[j],cross[i].oroad[tmp]);
		}
	
	//对路口排序 
	sort(&cross[0],&cross[cross.size()],cmp1);
	
	//初始化时间和结束标记 
	now=0;
	v=false;
	
}

//更新1个时间片 
void SM::update(){
	if(v)
		return;
	++now;

	for(SM_ROAD &i:road)//在道路内部更新 
		i.update1();
	for(SM_CROSS &i:cross)//更新路口 
		i.update2();
	for(SM_ROAD &i:road)//让要出发的车出发 
		i.update3();
	
	//统计死锁的车 
	error.clear();
	for(SM_ROAD &i:road)
		if(i.lst){
			error.push_back(i.pass[i.now].front());
			break;
		}
	
	//检查是否跑完 
	v=true;
	for(SM_CAR &i:car)
		if(i.sta!=2){
			v=false;
			break;
		}
}

//设置或更新一辆车的路径 
void SM::carrun(int id,const vector<int> &path){
	SM_CAR *tmp=carmap[id];
	tmp->path=path;
	tmp->now=0;
	if(tmp->sta==0){
		SM_CROSS *tar=crossmap[path[0]];
		++tmp->sta;
		for(int i=0;i<4;++i)
			if(tar->oroad[i]->tid==path[1])
				tar->oroad[i]->wait.push_back(tmp);
	}
}
