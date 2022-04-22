#include <math.h>
#include "stdio.h"
#include "stdlib.h"

#define MaxLength 50
#define Height 10
#define Width 10

//节点属性
#define Reachable 0	  //可以到达的节点
#define Bar 1		  //障碍物
#define Pass 2		  //
#define Source 3	  //起点
#define Destination 4 //终点

// 8bits分别代表相邻8个节点是否可到达
#define North (1 << 0)
#define North_West (1 << 1)
#define West (1 << 2)
#define South_West (1 << 3)
#define South (1 << 4)
#define South_East (1 << 5)
#define East (1 << 6)
#define North_East (1 << 7)

typedef struct //坐标
{
	int x, y;
} Point;



typedef struct //节点
{
	int x, y;	   //节点坐标
	int reachable; //节点属性
	int sur;	   //相邻节点的方位
	int value;
} MapNode;

typedef struct Close // list链表成员
{
	MapNode *cur;		//节点
	char vis;			//有效值，是否已被放入close list
	struct Close *from; //父节点 链表成员
	int F, G;
	int H;
	int dir; //下一个方位
} Close;

typedef struct //优先队列（Open表）
{
	int length;				 //当前队列的长度
	Close *Array[MaxLength]; //评价结点的指针
} Open;

void FindPath(int *way, int sx, int sy, int dx, int dy);
