#include "AStarRoute.h"



int srcX, srcY, dstX, dstY;

MapNode graph[Height][Width];	  //地图节点
Close close[Height][Width] = {0}; //地图成员

const Point dir[4] = //上北下南左西右东
	{
		{-1, 0}, // North 				//向北移动即x-1
		{0, -1}, // West
		{1, 0},	 // South
		{0, 1},	 // East
};
	
/* 导入一张地图，设置地图各节点属性，及其可移动方向*/
void initGraph(int map[Height][Width], int sx, int sy, int dx, int dy)
{
	int i, j;
	srcX = sx; //起点X坐标
	srcY = sy; //起点Y坐标
	dstX = dx; //终点X坐标
	dstY = dy; //终点Y坐标
	for (i = 0; i < Height; i++)
	{
		for (j = 0; j < Width; j++)
		{
			graph[i][j].x = i; //地图坐标X
			graph[i][j].y = j; //地图坐标Y
			graph[i][j].value = map[i][j];
			graph[i][j].reachable = (graph[i][j].value == Reachable); // 节点是否可到达
			graph[i][j].sur = 0;									  //可到达邻接节点位置
			if (!graph[i][j].reachable)
			{
				continue;
			}
			//设置除边框外，可到达邻接节点位置
			if (j > 0)
			{
				if (graph[i][j - 1].reachable) // left节点可到达
				{
					graph[i][j].sur |= West;
					graph[i][j - 1].sur |= East;
				}
				// if (i > 0)
				// {
				// 	if (graph[i - 1][j - 1].reachable && graph[i - 1][j].reachable && graph[i][j - 1].reachable) // up-left节点可到达
				// 	{
				// 		graph[i][j].sur |= North_West;
				// 		graph[i - 1][j - 1].sur |= South_East;
				// 	}
				// }
			}
			if (i > 0)
			{
				if (graph[i - 1][j].reachable) // up节点可到达
				{
					graph[i][j].sur |= North;
					graph[i - 1][j].sur |= South;
				}
				// if (j < Width - 1)
				// {
				// 	if (graph[i - 1][j + 1].reachable && graph[i - 1][j].reachable && map[i][j + 1] == Reachable) // up-right节点可到达
				// 	{
				// 		graph[i][j].sur |= North_East;
				// 		graph[i - 1][j + 1].sur |= South_West;
				// 	}
				// }
			}
		}
	}
}

//地图节点成员初始化操作
//导入起点及终点
void initClose(Close cls[Height][Width], int sx, int sy, int dx, int dy)
{
	// 地图Close表初始化配置
	int i, j;
	for (i = 0; i < Height; i++)
	{
		for (j = 0; j < Width; j++)
		{
			cls[i][j].cur = &graph[i][j];			// Close表所指节点
			cls[i][j].vis = !graph[i][j].reachable; // 能否被访问
			cls[i][j].from = NULL;					// 父节点
			cls[i][j].G = cls[i][j].F = 0;
			cls[i][j].H = 10 * abs(dx - i) + 10 * abs(dy - j); // 扩大十倍，避免浮点运算
		}
	}
	cls[sx][sy].F = cls[sx][sy].H; //起始点评价初始值
	cls[sy][sy].G = 0;			   //移步花费代价值
								   // cls[dx][dy].G = Infinity;
}

// Open表初始化
void initOpen(Open *q) //优先队列初始化
{
	q->length = 0; // 队内成员数初始为0
}

/*向优先队列（Open表）中添加成员
   并排序*/
void push(Open *q, Close cls[Height][Width], int x, int y, int g)
{
	Close *t;
	int i, mintag;

	cls[x][y].G = g; //所添加节点的坐标
	cls[x][y].F = cls[x][y].G + cls[x][y].H;

	q->Array[q->length++] = &(cls[x][y]);
	mintag = q->length - 1;
	for (i = 0; i < q->length - 1; i++) //确认最F值最小的节点
	{
		if (q->Array[i]->F < q->Array[mintag]->F)
		{
			mintag = i;
		}
	}
	t = q->Array[q->length - 1];
	q->Array[q->length - 1] = q->Array[mintag];
	q->Array[mintag] = t; //将F值最小节点置于表头
}

//取出Open list表中的F值最小的成员
Close *shift(Open *q)
{
	return q->Array[--q->length];
}

Open q;	  // Open表
Close *p; // list表成员
Close *astar()
{								   // A*算法遍历
	int i, curX, curY, surX, surY; //当前节点坐标，目标节点坐标
	int surG;

	initOpen(&q);
	initClose(close, srcX, srcY, dstX, dstY); //导入起点 和 终点，并将起点设置为成员
	close[srcX][srcY].vis = 1;
	push(&q, close, srcX, srcY, 0); //起点放入Close list，设置为不可访问

	while (q.length)
	{
		p = shift(&q);
		curX = p->cur->x;
		curY = p->cur->y;
		if (!p->H)
		{
			return p;
		}
		for (i = 0; i < 4; i++)
		{
			if (!(p->cur->sur & (1 << i * 2)))
			{
				continue;
			}
			surX = curX + dir[i].x;
			surY = curY + dir[i].y;

			// surG = p->G + sqrt((curX - surX) * (curX - surX) + (curY - surY) * (curY - surY));
			// surG = p->G + abs(curX - surX) + abs(curY - surY);
			//将距离扩大十倍，避免浮点运算
			if (abs(dir[i].x) > 0 && abs(dir[i].y) > 0)
				surG = p->G + 14; // 1.414
			else
				surG = p->G + 10;
			if (!close[surX][surY].vis) //当前节点成员不在Openlist中
			{
				close[surX][surY].vis = 1; //放入
				close[surX][surY].from = p;
				close[surX][surY].dir = (i + 2) % 4 + 1;
				push(&q, close, surX, surY, surG);
			}
			else
			{
				if (surG <= close[surX][surY].G) // Openlist中已经存在时，如果G更小时，更新当前节点成员
				{
					close[surX][surY].vis = 1;
					close[surX][surY].from = p;
					close[surX][surY].dir = (i + 2) % 4 + 1;
					push(&q, close, surX, surY, surG);
				}
			}
		}
	}
	return 0; //无结果
}

void FindPath(int *way, int sx, int sy, int dx, int dy)
{
	int map[10][10] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	};

	initGraph(map, sx, sy, dx, dy);
	Close *t = astar();
	Close *ret = t;
	int len = 0;

	while (ret != NULL)
	{
		//printf("from:%d,%d  dir:%d\n", ret->cur->x, ret->cur->y, ret->dir);
		ret = ret->from;
		len++;
	}
	// int route[len][3];
	way[len]=-1;
	for (int i = len - 1; i >= 0; i--)
	{
		// route[i][0] = t->cur->x;
		// route[i][1] = t->cur->y;
		way[i] = t->dir;
		t = t->from;
	}
}
